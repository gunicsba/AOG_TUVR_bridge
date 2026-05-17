"""AgOpenGPS <-> TUVR variable-rate controller bridge.

Reads section / speed PGNs from AgIO over UDP, drives a binary serial link
to a TUVR-speaking controller at 38400 8-N-1, and feeds the controller's
section state back into AgIO so the rate controller UI mirrors reality.
"""
import logging
import logging.handlers
import msvcrt
import os
import socket
import sys
import threading
import time
from configparser import ConfigParser
from enum import Enum, auto
from typing import List, Optional

import serial
import serial.tools.list_ports

from tuvr_protocol import (
    FUNCTION,
    SECTION_COUNT_CHANGE,
    StreamParser,
    build_gps_speed_cmd,
    build_section_state_cmd,
    build_section_state_req,
    build_status_req,
    unpack_section_bits,
)

# ---------------------------------------------------------------------------
#  Serial / protocol constants
# ---------------------------------------------------------------------------
BAUD = 38400
DEFAULT_SECTION_COUNT = 8

# ---------------------------------------------------------------------------
#  Timing
# ---------------------------------------------------------------------------
DEFAULT_SCT_HZ = 5             # SECTION_STATE_CMD rate
DEFAULT_SPD_HZ = 5             # GPS_SPEED_CMD rate
DEFAULT_STATUS_HZ = 1          # STATUS_REQ rate

MACHINE_TIMEOUT_S = 5.0        # No valid controller packet -> DISCONNECTED
SPEED_VALIDITY_WINDOW_S = 3.0  # Speed source=1 only if AgIO spoke within this

TICK_S = 0.05                   # Periodic loop granularity (50 ms)

# ---------------------------------------------------------------------------
#  UDP / AgIO
# ---------------------------------------------------------------------------
UDP_PORT = 8888
UDP_TIMEOUT_S = 3
AOG_PORT = 9999
AOG_MACHINE_SRC = 0x7B          # 123 = machine module
AOG_ISOBUS_SRC = 0x80           # 128 = ISOBUS / Task Controller source

# ---------------------------------------------------------------------------
#  Logging
# ---------------------------------------------------------------------------
LOG_LEVEL = logging.INFO
LOG_FMT = "[%(asctime)s.%(msecs)03d] %(levelname)s %(message)s"
LOG_DATEFMT = "%H:%M:%S"

logging.basicConfig(
    level=LOG_LEVEL,
    format=LOG_FMT,
    datefmt=LOG_DATEFMT,
)
logger = logging.getLogger("tuvr")


# ---------------------------------------------------------------------------
#  Config
# ---------------------------------------------------------------------------
def get_app_directory() -> str:
    """Return the directory containing the script or frozen exe."""
    if getattr(sys, "frozen", False):
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.abspath(__file__))


def _app_basename() -> str:
    """Return the base name of the running exe/script (no extension)."""
    if getattr(sys, "frozen", False):
        return os.path.splitext(os.path.basename(sys.executable))[0]
    return os.path.splitext(os.path.basename(__file__))[0]


CONFIG_PATH = os.path.join(get_app_directory(), _app_basename() + ".ini")
LOG_PATH = os.path.join(get_app_directory(), _app_basename() + ".log")


def load_config() -> ConfigParser:
    config = ConfigParser()
    if not os.path.exists(CONFIG_PATH):
        config["main"] = {
            "com": "0",
            "comms_lost_zero": "1",
            "sections": str(DEFAULT_SECTION_COUNT),
            "sct_hz": str(DEFAULT_SCT_HZ),
            "spd_hz": str(DEFAULT_SPD_HZ),
            "status_hz": str(DEFAULT_STATUS_HZ),
            "subnet": "255.255.255.255",
            # 0 = always tell AOG master=ON (default, current behaviour)
            # 1 = mirror the controller's master-switch bit into SwitchPGN.main
            #     so AOG only paints when the implement reports itself working
            "use_implement_master": "0",
            # 1 = broadcast PGN 0xF0 every PGN 0xEF tick with the
            #     controller-reported actual section state (default).
            #     AOG uses this when the ISOBUS / Task Controller plugin
            #     is enabled in AgIO; otherwise it is silently ignored.
            # 0 = do not transmit the ISOBUS PGN 0xF0 feedback packet.
            "send_isobus_feedback": "1",
        }
        with open(CONFIG_PATH, "w") as f:
            config.write(f)
    else:
        config.read(CONFIG_PATH)
    return config


def save_config(config: ConfigParser) -> None:
    with open(CONFIG_PATH, "w") as f:
        config.write(f)


# ===========================================================================
#  Serial-level state machine
# ===========================================================================
class MachineState(Enum):
    DISCONNECTED = auto()   # No controller reply yet / timed out.
    READY = auto()          # Controller is alive; waiting for AgIO.
    RUNNING = auto()        # READY + AgIO connected.


# ===========================================================================
#  AgOpenGPS helpers
# ===========================================================================

def aog_checksum(msg: bytes) -> int:
    """Sum bytes 2..n-1 (everything between preamble and CRC slot)."""
    return sum(msg[2:]) & 0xFF


def build_hello_reply(relay_lo: int, relay_hi: int) -> bytes:
    """Build the Hello reply that makes the machine icon go green in AOG."""
    msg = bytearray([
        0x80, 0x81,
        AOG_MACHINE_SRC,
        AOG_MACHINE_SRC,
        5,
        relay_lo & 0xFF,
        relay_hi & 0xFF,
        0, 0, 0,
    ])
    msg.append(aog_checksum(msg))
    return bytes(msg)


def build_from_machine(relay_lo: int, relay_hi: int) -> bytes:
    """Build the 'From Machine' PGN 0xED."""
    msg = bytearray([
        0x80, 0x81,
        AOG_MACHINE_SRC,
        0xED,
        8,
        relay_lo & 0xFF,
        relay_hi & 0xFF,
        0, 0,
        0, 0, 0, 0,
    ])
    msg.append(aog_checksum(msg))
    return bytes(msg)


def build_section_data(main_sw_bits: int, relay_lo: int, relay_hi: int,
                       off_lo: int = 0, off_hi: int = 0) -> bytes:
    """PGN 0xEA (234) -- Section Control Data back to AgIO."""
    msg = bytearray([
        0x80, 0x81,
        AOG_MACHINE_SRC,
        0xEA,
        8,
        main_sw_bits & 0xFF,
        0, 0, 0,
        relay_lo & 0xFF,
        off_lo & 0xFF,
        relay_hi & 0xFF,
        off_hi & 0xFF,
    ])
    msg.append(aog_checksum(msg))
    return bytes(msg)


def build_isobus_section_feedback(enabled: bool, count: int,
                                  bits: List[bool]) -> bytes:
    """PGN 0xF0 -- ISOBUS Task Controller -> AOG actual section state.

    Mirrors the AOG-TaskController format so AOG (with ISOBUS plugin
    enabled in AgIO) treats this as the authoritative actual-state
    feedback channel, independent of the Machine PGN 0xEA path.

    Layout:
      data[0]   = section control enabled flag (0/1)
      data[1]   = number of sections (1..255)
      data[2..] = packed section bits, 8 sections per byte, LSB-first
                  (bit i of byte k -> section k*8 + i + 1)
    """
    n = max(1, min(255, count))
    padded = list(bits)
    if len(padded) < n:
        padded += [False] * (n - len(padded))
    padded = padded[:n]

    payload = bytearray([1 if enabled else 0, n])
    i = 0
    while i < n:
        b = 0
        for k in range(8):
            if i + k < n and padded[i + k]:
                b |= (1 << k)
        payload.append(b)
        i += 8

    msg = bytearray([0x80, 0x81, AOG_ISOBUS_SRC, 0xF0, len(payload)])
    msg.extend(payload)
    msg.append(aog_checksum(msg))
    return bytes(msg)


def _crc8(data: bytes, length: int) -> int:
    return sum(data[:length]) & 0xFF


def build_switch_pgn(auto_on: bool, master_on: bool,
                     sw_lo: int, sw_hi: int) -> bytes:
    """PGN 32618 -- switch-box feedback to Rate Controller."""
    flags = 0
    if auto_on:
        flags |= 0x01
    if master_on:
        flags |= 0x02
    else:
        flags |= 0x04

    msg = bytearray([
        106,
        127,
        flags,
        sw_lo & 0xFF,
        sw_hi & 0xFF,
        0,
    ])
    msg[5] = _crc8(msg, 5)
    return bytes(msg)


# ===========================================================================
#  COM port selection
# ===========================================================================

def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No COM ports available.")
        return []

    print("Available COM ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  ({p.description})")
    return ports


def select_port() -> Optional[str]:
    ports = list_ports()
    if not ports:
        return None

    while True:
        choice = input("Select port index or COM name: ").strip()

        if choice.isdigit():
            idx = int(choice)
            if 0 <= idx < len(ports):
                return ports[idx].device

        if choice.upper().startswith("COM"):
            return choice.upper()

        print("Invalid choice.")


# ===========================================================================
#  TUVRRequester -- serial-side state machine + scheduler
# ===========================================================================

class TUVRRequester:
    def __init__(self, ser: serial.Serial, section_count: int,
                 sct_hz: int, spd_hz: int, status_hz: int,
                 use_implement_master: bool,
                 send_isobus_feedback: bool,
                 config: ConfigParser) -> None:
        self.ser = ser
        self.config = config
        self.use_implement_master = use_implement_master
        self.send_isobus_feedback = send_isobus_feedback
        # Last decoded controller master-switch bit; None until first STATUS_RESP
        self.controller_master_on: Optional[bool] = None
        # Last decoded operational-mode bitfield from STATUS_RESP (16-bit)
        self.controller_opmode: Optional[int] = None
        # Last decoded physical-section-switch bits from STATUS_RESP (32-bit)
        self.controller_phys_switches: int = 0
        # Last AOG PGN 0xEF signal bytes (uturn, speed, hyd, tram, geo) for
        # change-detection on diagnostic logging.
        self.last_ef_signal: Optional[bytes] = None
        self.lock = threading.Lock()            # serial write lock
        self.sections_lock = threading.Lock()   # target_sections / speed
        self.running = True

        # Connection state
        self.state = MachineState.DISCONNECTED
        self.last_valid_machine_time = 0.0

        # Configured limits
        self.section_count = max(1, min(16, section_count))

        # Section state
        self.target_sections = [0] * self.section_count          # from AOG
        self.machine_sections: Optional[List[int]] = None         # last from controller

        # Speed (km/h) from AOG
        self.current_speed_kmh = 0.0
        self.last_aog_speed_time = 0.0

        # AgIO connection
        self.agio_connected = False

        # Relay bytes for PGNs back to AOG
        self.relay_lo = 0
        self.relay_hi = 0
        self.off_lo = 0
        self.off_hi = 0
        self.main_sw_bits = 0
        self.is_auto_mode = True       # always-auto in V1
        self.switch_pgn_pending: Optional[bytes] = None
        self._last_sw_pgn: Optional[bytes] = None

        # Periodic timers
        self.sct_hz = max(1, sct_hz)
        self.spd_hz = max(1, spd_hz)
        self.status_hz = max(1, status_hz)
        self.last_status_time = 0.0
        self.last_sct_time = 0.0
        self.last_spd_time = 0.0

    # ------------------------------------------------------------------
    #  Serial write helpers
    # ------------------------------------------------------------------
    def _send(self, packet: bytes, label: str) -> None:
        with self.lock:
            self.ser.write(packet)
            self.ser.flush()
        logger.info(f"TX >> {label} [{packet.hex()}]")

    # ------------------------------------------------------------------
    #  State transitions
    # ------------------------------------------------------------------
    def enter_disconnected(self, reason: str) -> None:
        if self.state != MachineState.DISCONNECTED:
            logger.info(f"STATE -> DISCONNECTED ({reason})")
        self.state = MachineState.DISCONNECTED
        self.machine_sections = None

    def enter_ready(self, reason: str) -> None:
        if self.state != MachineState.READY:
            logger.info(f"STATE -> READY ({reason})")
        self.state = MachineState.READY

    def enter_running(self, reason: str) -> None:
        if self.state != MachineState.RUNNING:
            logger.info(f"STATE -> RUNNING ({reason})")
        self.state = MachineState.RUNNING
        # Fire SCT/SPD immediately on entering RUNNING.
        self.last_sct_time = 0.0
        self.last_spd_time = 0.0

    # ------------------------------------------------------------------
    #  Periodic scheduler
    # ------------------------------------------------------------------
    def periodic_loop(self) -> None:
        while self.running:
            now = time.time()

            # --- timeout check ---
            if self.state != MachineState.DISCONNECTED:
                if self.last_valid_machine_time > 0 and \
                   (now - self.last_valid_machine_time) > MACHINE_TIMEOUT_S:
                    self.enter_disconnected(
                        f"machine timeout "
                        f"{now - self.last_valid_machine_time:.1f}s")

            # READY -> RUNNING when AgIO connects
            if self.state == MachineState.READY and self.agio_connected:
                self.enter_running("AgIO connected")

            # --- STATUS_REQ heartbeat (all states; acts as probe when DISCONNECTED) ---
            if (now - self.last_status_time) >= (1.0 / self.status_hz):
                self._send(build_status_req(), "STATUS_REQ")
                self.last_status_time = now

            # --- RUNNING-only sends ---
            if self.state == MachineState.RUNNING:
                if (now - self.last_sct_time) >= (1.0 / self.sct_hz):
                    with self.sections_lock:
                        bits = [bool(s) for s in self.target_sections]
                    n = self.section_count
                    if len(bits) < n:
                        bits = bits + [False] * (n - len(bits))
                    else:
                        bits = bits[:n]
                    self._send(build_section_state_cmd(bits),
                               f"SECTION_STATE_CMD {_bits_to_str(bits)}")
                    self.last_sct_time = now

                if (now - self.last_spd_time) >= (1.0 / self.spd_hz):
                    with self.sections_lock:
                        spd_kmh = self.current_speed_kmh
                        last_spd = self.last_aog_speed_time
                    mm_per_s = int(round(spd_kmh * 1000.0 / 3.6))
                    source = 1 if (now - last_spd) < SPEED_VALIDITY_WINDOW_S else 0
                    self._send(
                        build_gps_speed_cmd(mm_per_s, source),
                        f"GPS_SPEED_CMD {spd_kmh:.1f}km/h "
                        f"({mm_per_s}mm/s src={source})")
                    self.last_spd_time = now

            time.sleep(TICK_S)

    # ------------------------------------------------------------------
    #  Updates from AgOpenGPS
    # ------------------------------------------------------------------
    def update_sections_from_aog(self, section_bits: int) -> None:
        new_sections = [0] * self.section_count
        for i in range(min(8, self.section_count)):
            new_sections[i] = 1 if (section_bits >> i) & 1 else 0

        changed = False
        with self.sections_lock:
            if new_sections != self.target_sections:
                changed = True
            self.target_sections = new_sections

        # Send immediately on change so the controller reacts without waiting.
        if changed and self.state == MachineState.RUNNING:
            n = self.section_count
            bits = [bool(s) for s in new_sections]
            if len(bits) < n:
                bits = bits + [False] * (n - len(bits))
            else:
                bits = bits[:n]
            self._send(build_section_state_cmd(bits),
                       f"SECTION_STATE_CMD (change) {_bits_to_str(bits)}")

    def update_speed_from_aog(self, speed_kmh: float) -> None:
        with self.sections_lock:
            self.current_speed_kmh = speed_kmh
            self.last_aog_speed_time = time.time()

    # ------------------------------------------------------------------
    #  Incoming packet dispatch
    # ------------------------------------------------------------------
    def handle_packet(self, id_byte: int, function: int,
                      payload: bytes) -> None:
        """Called by the receiver thread for every validated frame."""
        self.last_valid_machine_time = time.time()

        # First valid frame in any state brings us up to READY.
        if self.state == MachineState.DISCONNECTED:
            self.enter_ready("first valid frame")

        if function == FUNCTION.STATUS:
            self._on_status(payload)
        elif function == FUNCTION.SECTION_STATE:
            self._on_section_state(payload)
        elif function == FUNCTION.GPS_SPEED:
            logger.debug(f"GPS_SPEED echo [{payload.hex()}]")
        else:
            logger.info(
                f"RX unhandled function 0x{function:02X} [{payload.hex()}]")

    # ---- STATUS_RESP handler ----
    def _on_status(self, payload: bytes) -> None:
        """Decode the controller's master/physical-switch bits.

        payload layout (after header/ID/SubID/PageID strip):
          [0]   transaction id
          [1]   boom equipment state (bit 0 = master switch ON)
          [2-3] current operational mode bits
          [4-7] physical section switch states (32-bit, LE)
          [8-11] applied rate
          [12-13] last error
        """
        if len(payload) < 8:
            logger.debug(f"STATUS reply (short) [{payload.hex()}]")
            return

        boom = payload[1]
        master_on = bool(boom & 0x01)
        opmode = int.from_bytes(payload[2:4], "little", signed=False)
        phys = int.from_bytes(payload[4:8], "little", signed=False)

        if master_on != self.controller_master_on:
            logger.info(
                f"Controller master switch -> {'ON' if master_on else 'OFF'}")
            self.controller_master_on = master_on
            # Push an updated SwitchPGN so AOG sees the change immediately
            # (only relevant when we're mirroring it).
            if self.use_implement_master:
                self._refresh_switch_pgn()

        if opmode != self.controller_opmode:
            flags = []
            if opmode & 0x0001:
                flags.append("AUTO_RATE")
            if opmode & 0x0002:
                flags.append("AUTO_SECTION")
            if opmode & 0x0004:
                flags.append("MASTER_SW")
            if opmode & 0x0008:
                flags.append("AUX_VALVE")
            if opmode & 0x0010:
                flags.append("AUTOSTEER")
            label = ",".join(flags) if flags else "-"
            logger.info(
                f"Controller op-mode -> 0x{opmode:04X} [{label}]")
            self.controller_opmode = opmode

        if phys != self.controller_phys_switches:
            logger.info(
                f"Controller physical section switches = 0x{phys:08X}")
            self.controller_phys_switches = phys

    def _refresh_switch_pgn(self) -> None:
        """Rebuild SwitchPGN with the current relay bytes + master authority."""
        main_on = True
        if (self.use_implement_master
                and self.controller_master_on is not None):
            main_on = self.controller_master_on
        new_sw_pgn = build_switch_pgn(
            True, main_on, self.relay_lo, self.relay_hi)
        if new_sw_pgn != self._last_sw_pgn:
            self._last_sw_pgn = new_sw_pgn
            self.switch_pgn_pending = new_sw_pgn

    # ---- SECTION_STATE handler ----
    def _on_section_state(self, payload: bytes) -> None:
        try:
            count, bits = unpack_section_bits(payload)
        except ValueError as e:
            logger.warning(f"SECTION_STATE parse error: {e}")
            return

        if count == SECTION_COUNT_CHANGE:
            logger.debug("SECTION_STATE data-change sentinel received")
            return

        if count != self.section_count:
            logger.info(
                f"SECTION_STATE count {count} != "
                f"configured {self.section_count}")

        # Fit the bridge's configured section_count for AOG bitmask purposes.
        cur = [1 if (i < len(bits) and bits[i]) else 0
               for i in range(self.section_count)]
        if cur != self.machine_sections:
            self.machine_sections = cur
            logger.info(f"Controller sections = {cur}")

        # Update relay/off masks for AgOpenGPS feedback PGNs.
        relay = 0
        off = 0
        for i, on in enumerate(cur):
            if on:
                relay |= (1 << i)
            else:
                off |= (1 << i)
        self.relay_lo = relay & 0xFF
        self.relay_hi = (relay >> 8) & 0xFF
        self.off_lo = off & 0xFF
        self.off_hi = (off >> 8) & 0xFF

        # Master-switch authority: hardcoded ON, or mirrored from controller
        # if use_implement_master is enabled in the config.
        self._refresh_switch_pgn()


def _bits_to_str(bits: List[bool]) -> str:
    return "".join("1" if b else "0" for b in bits)


# ===========================================================================
#  Thread functions
# ===========================================================================

def receiver_loop(ser: serial.Serial, parser: StreamParser,
                  req: TUVRRequester) -> None:
    """Reads serial bytes, hands validated frames to the requester."""
    while req.running:
        try:
            data = ser.read(256)
            if not data:
                continue

            for id_byte, function, payload in parser.feed(data):
                logger.info(
                    f"RX << id=0x{id_byte:02X} "
                    f"fn=0x{function:02X} [{payload.hex()}]")
                req.handle_packet(id_byte, function, payload)

        except Exception as e:
            logger.info(f"RX error: {e}")
            time.sleep(0.2)


def udp_listener_loop(req: TUVRRequester, comms_lost_zero: bool,
                      subnet: str) -> None:
    """Receives AgOpenGPS PGNs and sends feedback PGNs back."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("", UDP_PORT))
    sock.settimeout(UDP_TIMEOUT_S)
    logger.info(f"UDP listening on port {UDP_PORT}")

    broadcast = (subnet, AOG_PORT)
    logger.info(f"UDP broadcast -> {broadcast}")

    while req.running:
        try:
            data, addr = sock.recvfrom(1024)
        except socket.timeout:
            if req.agio_connected:
                logger.info("AgIO timeout -- connection lost")
                req.agio_connected = False
                if comms_lost_zero:
                    req.update_sections_from_aog(0x00)
                    req.update_speed_from_aog(0.0)
                if req.state == MachineState.RUNNING:
                    req.enter_ready("AgIO timeout")
            continue
        except OSError:
            if not req.running:
                break
            raise

        if len(data) < 5:
            continue
        if data[0] != 0x80 or data[1] != 0x81:
            continue

        pgn = data[3]

        if pgn == 0xC8:  # AgIO Hello
            if not req.agio_connected:
                version = data[5] if len(data) > 5 else 0
                logger.info(f"AgIO connected (version {version / 10:.1f})")
            req.agio_connected = True

            if req.state in (MachineState.READY, MachineState.RUNNING):
                reply = build_hello_reply(req.relay_lo, req.relay_hi)
                sock.sendto(reply, broadcast)
                logger.debug(
                    f"TX Hello reply -> {broadcast} [{reply.hex()}]")
            else:
                logger.debug("AgIO Hello ignored -- controller not ready")

        elif pgn == 0xEF:  # Machine Data -- section bits
            if len(data) > 12:
                section_bits = data[11]
                req.update_sections_from_aog(section_bits)

                # Diagnostic: log AOG control bytes (tramline, hyd-lift,
                # uturn, geoStop) whenever they change.  Byte positions
                # follow the canonical AgIO Machine Data PGN layout.
                ef_signal = bytes(data[5:10])  # uturn, speed, hyd, tram, geo
                if ef_signal != req.last_ef_signal:
                    if (req.last_ef_signal is None
                            or ef_signal[0] != req.last_ef_signal[0]):
                        logger.info(f"AOG uTurn -> 0x{ef_signal[0]:02X}")
                    if (req.last_ef_signal is None
                            or ef_signal[2] != req.last_ef_signal[2]):
                        hname = {0: "none", 1: "LOWER", 2: "RAISE"}.get(
                            ef_signal[2], f"0x{ef_signal[2]:02X}")
                        logger.info(f"AOG hyd-lift -> {hname}")
                    if (req.last_ef_signal is None
                            or ef_signal[3] != req.last_ef_signal[3]):
                        tname = {0: "off", 1: "RIGHT",
                                 2: "LEFT", 3: "BOTH"}.get(
                            ef_signal[3], f"0x{ef_signal[3]:02X}")
                        logger.info(f"AOG TRAMLINE -> {tname}")
                    if (req.last_ef_signal is None
                            or ef_signal[4] != req.last_ef_signal[4]):
                        logger.info(
                            f"AOG geoStop -> 0x{ef_signal[4]:02X}")
                    req.last_ef_signal = ef_signal

                if req.state == MachineState.RUNNING:
                    if req.is_auto_mode:
                        # Auto mode: AOG drives sections.  Do NOT feed
                        # back the controller's stale relay/off state --
                        # that causes a race where AOG sees "off" before
                        # the controller has confirmed "on" and immediately
                        # reverts its own command.
                        ea_relay_lo, ea_relay_hi = 0, 0
                        ea_off_lo, ea_off_hi = 0, 0
                    else:
                        ea_relay_lo = req.relay_lo
                        ea_relay_hi = req.relay_hi
                        ea_off_lo = req.off_lo
                        ea_off_hi = req.off_hi
                    sect_data = build_section_data(
                        req.main_sw_bits,
                        ea_relay_lo, ea_relay_hi,
                        ea_off_lo, ea_off_hi)
                    sock.sendto(sect_data, broadcast)
                    req.main_sw_bits = 0

                    from_machine = build_from_machine(
                        ea_relay_lo, ea_relay_hi)
                    sock.sendto(from_machine, broadcast)

                    sw_pgn = req.switch_pgn_pending
                    if sw_pgn is not None:
                        sock.sendto(sw_pgn, broadcast)
                        req.switch_pgn_pending = None
                        logger.info(
                            f"TX SwitchPGN -> {broadcast} [{sw_pgn.hex()}]")

                    # Optional ISOBUS-style actual-state feedback (PGN 0xF0).
                    # Sent every PGN 0xEF tick (~10 Hz) with the controller's
                    # last-reported section state, matching the cadence used
                    # by AOG-TaskController.
                    if (req.send_isobus_feedback
                            and req.machine_sections is not None):
                        bits = [bool(s) for s in req.machine_sections]
                        iso = build_isobus_section_feedback(
                            True, req.section_count, bits)
                        sock.sendto(iso, broadcast)

        elif pgn == 0xFE:  # Steer Data -- speed + section bits
            if len(data) > 6:
                spd = int.from_bytes(data[5:7], "little", signed=False) * 0.1
                req.update_speed_from_aog(spd)
            if len(data) > 12:
                section_bits = data[11]
                req.update_sections_from_aog(section_bits)


def keyboard_loop(req: TUVRRequester) -> None:
    logger.info("Keyboard: X = exit")
    while req.running:
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key in (b"x", b"X"):
                req.running = False
                logger.info("Exit requested")
                break
        time.sleep(0.05)


# ===========================================================================
#  Main
# ===========================================================================

def _setup_file_logging() -> None:
    """Add a rotating file handler so logs survive console close."""
    handler = logging.handlers.RotatingFileHandler(
        LOG_PATH, maxBytes=2 * 1024 * 1024, backupCount=3,
        encoding="utf-8")
    handler.setLevel(LOG_LEVEL)
    handler.setFormatter(logging.Formatter(LOG_FMT, datefmt=LOG_DATEFMT))
    logger.addHandler(handler)
    logger.info(f"Logging to file: {LOG_PATH}")


def main() -> None:
    _setup_file_logging()
    print("AOG-TUVR Bridge  (AgOpenGPS <-> TUVR VR controller)")
    print()

    config = load_config()
    saved_com = config.get("main", "com", fallback="0")
    comms_lost_zero = config.getboolean("main", "comms_lost_zero", fallback=True)
    section_count = config.getint("main", "sections",
                                   fallback=DEFAULT_SECTION_COUNT)
    sct_hz = config.getint("main", "sct_hz", fallback=DEFAULT_SCT_HZ)
    spd_hz = config.getint("main", "spd_hz", fallback=DEFAULT_SPD_HZ)
    status_hz = config.getint("main", "status_hz", fallback=DEFAULT_STATUS_HZ)
    use_impl_master = config.getboolean(
        "main", "use_implement_master", fallback=False)
    send_isobus = config.getboolean(
        "main", "send_isobus_feedback", fallback=True)
    subnet = config.get("main", "subnet", fallback="255.255.255.255")

    print(f"Config: sections={section_count}  SCT={sct_hz}Hz  "
          f"SPD={spd_hz}Hz  STATUS={status_hz}Hz  "
          f"comms_lost_zero={comms_lost_zero}  "
          f"use_implement_master={use_impl_master}  "
          f"send_isobus_feedback={send_isobus}  subnet={subnet}")
    print()

    available = {p.device for p in serial.tools.list_ports.comports()}
    if saved_com != "0" and saved_com in available:
        print(f"Using saved COM port: {saved_com}")
        port = saved_com
    else:
        if saved_com != "0":
            print(f"Saved port {saved_com} not found.")
        port = select_port()
        if not port:
            return
        config.set("main", "com", port)
        save_config(config)

    logger.info(f"Opening {port} @ {BAUD} baud")

    ser = serial.Serial(
        port=port,
        baudrate=BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
    )

    parser = StreamParser()
    requester = TUVRRequester(ser, section_count, sct_hz, spd_hz,
                              status_hz, use_impl_master,
                              send_isobus, config)

    threads = [
        threading.Thread(target=udp_listener_loop,
                         args=(requester, comms_lost_zero, subnet),
                         daemon=True),
        threading.Thread(target=receiver_loop,
                         args=(ser, parser, requester), daemon=True),
        threading.Thread(target=requester.periodic_loop, daemon=True),
        threading.Thread(target=keyboard_loop, args=(requester,), daemon=True),
    ]
    for t in threads:
        t.start()

    try:
        while requester.running:
            time.sleep(0.2)
    except KeyboardInterrupt:
        requester.running = False
        logger.info("KeyboardInterrupt")
    finally:
        requester.running = False
        time.sleep(0.3)
        ser.close()
        logger.info("Serial port closed")


if __name__ == "__main__":
    main()
