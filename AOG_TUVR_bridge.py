import serial
import serial.tools.list_ports
import socket
import threading
import time
import msvcrt
import logging
import os
import sys
from configparser import ConfigParser
from typing import Optional, Tuple

# ---------------------------------------------------------------------------
#  HC5500 packet framing constants
# ---------------------------------------------------------------------------
SOH = 0x01
STX = 0x02
ETX = 0x03
EOT = 0x04

# ---------------------------------------------------------------------------
#  Timing / protocol constants
# ---------------------------------------------------------------------------
BAUD = 9600
BOOT_PERIOD_S = 1.0
RUN_PERIOD_S = 0.2
REQUEST_GAP_S = 0.05
SECTION_COUNT = 13
HC_TIMEOUT_S = 1

UDP_PORT = 8888
UDP_TIMEOUT_S = 3
AOG_PORT = 9999           # AgIO listens on this port for replies

# AgOpenGPS Machine Module identity
AOG_MACHINE_SRC = 0x7B    # 123 = machine module

# ---------------------------------------------------------------------------
#  Logging
# ---------------------------------------------------------------------------
LOG_LEVEL = logging.INFO

logging.basicConfig(
    level=LOG_LEVEL,
    format="[%(asctime)s.%(msecs)03d] %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("hc5500")

# ---------------------------------------------------------------------------
#  Config
# ---------------------------------------------------------------------------
def get_app_directory() -> str:
    """Get the application directory (works for both script and frozen exe)."""
    if getattr(sys, 'frozen', False):
        # Running as compiled executable
        return os.path.dirname(sys.executable)
    else:
        # Running as script
        return os.path.dirname(os.path.abspath(__file__))

CONFIG_PATH = os.path.join(get_app_directory(), "config.ini")

def load_config() -> ConfigParser:
    config = ConfigParser()
    if not os.path.exists(CONFIG_PATH):
        config["main"] = {"com": "0", "comms_lost_zero": "1"}
        with open(CONFIG_PATH, "w") as f:
            config.write(f)
    else:
        config.read(CONFIG_PATH)
    return config


def save_config(config: ConfigParser):
    with open(CONFIG_PATH, "w") as f:
        config.write(f)


# ===================================================================
#  HC5500 packet building / parsing  (unchanged from working DEMO)
# ===================================================================

def xor_checksum_ascii(header: str, payload: str) -> str:
    x = 0
    for ch in (header + payload):
        x ^= ord(ch)
    return f"{x:02X}"


def build_packet(header: str, payload: str) -> bytes:
    checksum = xor_checksum_ascii(header, payload)
    return (
        bytes([SOH])
        + header.encode("ascii")
        + bytes([STX])
        + payload.encode("ascii")
        + bytes([ETX])
        + checksum.encode("ascii")
        + bytes([EOT])
    )


def parse_packet(data: bytes) -> Optional[Tuple[str, str, str, str, bool]]:
    try:
        if len(data) < 8:
            return None
        if data[0] != SOH or data[-1] != EOT:
            return None

        stx_i = data.index(bytes([STX]))
        etx_i = data.index(bytes([ETX]))

        header = data[1:stx_i].decode("ascii", errors="replace")
        payload = data[stx_i + 1:etx_i].decode("ascii", errors="replace")
        checksum = data[etx_i + 1:etx_i + 3].decode("ascii", errors="replace")
        calc = xor_checksum_ascii(header, payload)
        valid = checksum.upper() == calc.upper()

        return header, payload, checksum, calc, valid
    except Exception:
        return None


class PacketStreamParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, chunk: bytes):
        self.buf.extend(chunk)
        items = []

        while True:
            try:
                start = self.buf.index(SOH)
            except ValueError:
                if len(self.buf) > 4096:
                    self.buf.clear()
                break

            if start > 0:
                garbage = bytes(self.buf[:start])
                items.append(("garbage", garbage))
                del self.buf[:start]

            try:
                end = self.buf.index(EOT, 1)
            except ValueError:
                break

            pkt = bytes(self.buf[:end + 1])
            del self.buf[:end + 1]
            items.append(("packet", pkt))

        return items


def hex_dump(data: bytes) -> str:
    return data.hex(" ").upper()


def ascii_dump(data: bytes) -> str:
    return "".join(chr(b) if 32 <= b <= 126 else "." for b in data)


# ===================================================================
#  AgOpenGPS message helpers
# ===================================================================

def aog_checksum(msg: bytes) -> int:
    """Sum bytes 2..n-1 (everything between preamble and CRC slot)."""
    return sum(msg[2:]) & 0xFF


def build_hello_reply(relay_lo: int, relay_hi: int) -> bytes:
    """Build the Hello reply that makes the machine icon go green in AOG.
    Format: 0x80 0x81 src=0x7B pgn=0x7B len=5 relayLo relayHi 0 0 0 CRC
    """
    msg = bytearray([
        0x80, 0x81,
        AOG_MACHINE_SRC,   # src  = 123
        AOG_MACHINE_SRC,   # pgn  = 123
        5,                 # len  = 5 data bytes
        relay_lo & 0xFF,
        relay_hi & 0xFF,
        0, 0, 0,
    ])
    msg.append(aog_checksum(msg))
    return bytes(msg)


def build_from_machine(relay_lo: int, relay_hi: int) -> bytes:
    """Build the 'From Machine' PGN 0xED sent every 200ms.
    Format: 0x80 0x81 src=0x7B pgn=0xED len=8 data[8] CRC
    """
    msg = bytearray([
        0x80, 0x81,
        AOG_MACHINE_SRC,   # src = 123
        0xED,              # pgn = 237 (From Machine)
        8,                 # len = 8 data bytes
        relay_lo & 0xFF,   # byte 5: relayLo
        relay_hi & 0xFF,   # byte 6: relayHi
        0, 0,              # bytes 7-8: reserved
        0, 0, 0, 0,        # bytes 9-12: reserved
    ])
    msg.append(aog_checksum(msg))
    return bytes(msg)


# ===================================================================
#  COM port selection
# ===================================================================

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


# ===================================================================
#  HCRequester  -- manages HC5500 serial communication
# ===================================================================

class HCRequester:
    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self.lock = threading.Lock()           # serial write lock
        self.sections_lock = threading.Lock()   # protects target_sections
        self.running = True

        # HC5500 connection state
        self.boot_mode = True
        self.last_valid_hc_time = 0.0

        # Section state (written by UDP thread, read by TX thread)
        self.target_sections = [0] * SECTION_COUNT
        self.last_hc_s6c = None
        self.last_hc_a6b = None

        # AgIO connection flag
        self.agio_connected = False

        # Current relay bytes (for AOG Hello reply / From Machine PGN)
        self.relay_lo = 0
        self.relay_hi = 0

    # ---- serial helpers ----

    def send_packet(self, header: str, payload: str):
        pkt = build_packet(header, payload)
        with self.lock:
            self.ser.write(pkt)
            self.ser.flush()
        logger.debug(f"TX {header} | {payload} | HEX={hex_dump(pkt)}")

    # ---- boot / run state machine ----

    def enter_boot_mode(self, reason: str):
        if not self.boot_mode:
            logger.info(f"STATE -> BOOT ({reason})")
        self.boot_mode = True

    def enter_run_mode(self, reason: str):
        if self.boot_mode:
            logger.info(f"STATE -> RUN ({reason})")
        self.boot_mode = False

    def send_boot_request(self):
        self.send_packet("R0D", "6A")

    def send_run_cycle(self):
        with self.sections_lock:
            sec = ",".join(str(x) for x in self.target_sections)

        self.send_packet("S0C", f"6C,{sec}")
        time.sleep(REQUEST_GAP_S)

        self.send_packet("R0D", "6B")
        time.sleep(REQUEST_GAP_S)

        self.send_packet("R0D", "6D")

    def periodic_loop(self):
        while self.running:
            now = time.time()

            if self.last_valid_hc_time > 0 and (now - self.last_valid_hc_time) > HC_TIMEOUT_S:
                self.enter_boot_mode(f"HC timeout {now - self.last_valid_hc_time:.2f}s")

            start = time.time()

            if self.boot_mode:
                self.send_boot_request()
                period = BOOT_PERIOD_S
            else:
                self.send_run_cycle()
                period = RUN_PERIOD_S

            elapsed = time.time() - start
            sleep_left = period - elapsed
            if sleep_left > 0:
                time.sleep(sleep_left)

    # ---- section update from AgOpenGPS ----

    def update_sections_from_aog(self, section_bits: int):
        """Map AgOpenGPS 8-bit section mask to HC5500 13-element array."""
        new_sections = [0] * SECTION_COUNT
        for i in range(8):
            new_sections[i] = 1 if (section_bits >> i) & 1 else 0
        with self.sections_lock:
            self.target_sections = new_sections
            self.relay_lo = section_bits & 0xFF

    # ---- HC5500 response parsing ----

    def parse_section_list(self, payload: str, record_id: str):
        parts = payload.split(",")
        if not parts or parts[0] != record_id:
            return None

        values = []
        for p in parts[1:1 + SECTION_COUNT]:
            try:
                values.append(int(p))
            except ValueError:
                return None

        if len(values) != SECTION_COUNT:
            return None

        return values

    def handle_valid_hc_packet(self, header: str, payload: str):
        self.last_valid_hc_time = time.time()
        self.enter_run_mode(f"valid HC packet {header}")

        first = payload.split(",", 1)[0] if payload else ""

        if header == "A0D" and first == "6A":
            logger.info(f"HC 6A config: {payload}")

        elif header == "A0D" and first == "69":
            try:
                scaled = float(payload.split(",")[1])
                hc_rate = scaled * 10000.0
                logger.debug(f"HC 69 target rate = {hc_rate:.1f} l/ha")
            except Exception:
                logger.info(f"HC 69 unexpected payload: {payload}")

        elif header == "S0C" and first == "68":
            try:
                scaled = float(payload.split(",")[1])
                logger.debug(f"HC S68 set/report rate = {scaled * 10000:.1f} l/ha")
            except Exception:
                logger.info(f"HC S68 unexpected payload: {payload}")

        elif header == "S0C" and first == "6C":
            values = self.parse_section_list(payload, "6C")
            if values is None:
                logger.info(f"HC S6C unexpected payload: {payload}")
            else:
                if values != self.last_hc_s6c:
                    self.last_hc_s6c = values
                    logger.info(f"HC S6C section state = {values}")
                else:
                    logger.debug(f"HC S6C section state unchanged = {values}")

        elif header == "A0D" and first == "6B":
            values = self.parse_section_list(payload[:-2] if payload.endswith(",A") else payload, "6B")
            self.last_hc_a6b = payload
            logger.debug(f"HC A6B desired sections = {payload}")

        elif header == "A0D" and first == "6D":
            logger.debug(f"HC 6D mode = {payload}")

        elif header == "V0C" and first == "68":
            try:
                scaled = float(payload.split(",")[1])
                logger.debug(f"HC V68 rate value = {scaled * 10000:.1f} l/ha")
            except Exception:
                logger.info(f"HC V68 unexpected payload: {payload}")

        elif header == "N0C" and first == "6C":
            values = self.parse_section_list(payload, "6C")
            logger.debug(f"HC N6C actual sections = {values if values is not None else payload}")

        else:
            logger.info(f"HC OTHER {header} | {payload}")


# ===================================================================
#  Thread functions
# ===================================================================

def receiver_loop(ser: serial.Serial, parser: PacketStreamParser, req: HCRequester):
    """Thread: reads serial data from HC5500, parses packets."""
    while req.running:
        try:
            data = ser.read(256)
            if not data:
                continue

            logger.debug(f"RAW HEX   {hex_dump(data)}")
            logger.debug(f"RAW ASCII {ascii_dump(data)}")

            for kind, blob in parser.feed(data):
                if kind == "garbage":
                    if blob:
                        logger.debug(f"GARBAGE HEX   {hex_dump(blob)}")
                        logger.debug(f"GARBAGE ASCII {ascii_dump(blob)}")
                    continue

                parsed = parse_packet(blob)
                if parsed is None:
                    logger.info(f"BADFRAME HEX   {hex_dump(blob)}")
                    logger.info(f"BADFRAME ASCII {ascii_dump(blob)}")
                    continue

                header, payload, checksum, calc, valid = parsed
                logger.debug(f"PKT valid={valid} cs={checksum} calc={calc} {header} | {payload}")

                if valid:
                    req.handle_valid_hc_packet(header, payload)

        except Exception as e:
            logger.info(f"RX error: {e}")
            time.sleep(0.2)


def udp_listener_loop(req: HCRequester, comms_lost_zero: bool):
    """Thread: receives AgOpenGPS PGNs via UDP, updates shared section state,
    and sends Hello reply + From Machine PGN back to AgIO."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("", UDP_PORT))
    sock.settimeout(UDP_TIMEOUT_S)
    logger.info(f"UDP listening on port {UDP_PORT}")

    # Broadcast address for replies to AgIO
    broadcast = ("255.255.255.255", AOG_PORT)

    while req.running:
        try:
            data, addr = sock.recvfrom(1024)
        except socket.timeout:
            if req.agio_connected:
                logger.info("AgIO timeout -- connection lost")
                req.agio_connected = False
                if comms_lost_zero:
                    req.update_sections_from_aog(0x00)
            continue
        except OSError:
            if not req.running:
                break
            raise

        if len(data) < 5:
            continue

        # Verify AOG preamble
        if data[0] != 0x80 or data[1] != 0x81:
            continue

        pgn = data[3]

        if pgn == 0xC8:  # AgIO Hello
            if not req.agio_connected:
                version = data[5] if len(data) > 5 else 0
                logger.info(f"AgIO connected (version {version / 10:.1f})")
            req.agio_connected = True

            # Only reply when HC5500 is alive (run mode) -- keeps icon red until connected
            if not req.boot_mode:
                reply = build_hello_reply(req.relay_lo, req.relay_hi)
                sock.sendto(reply, broadcast)
                logger.debug(f"TX Hello reply -> {broadcast}")
            else:
                logger.debug("AgIO Hello ignored -- HC5500 not connected (boot mode)")

        elif pgn == 0xEF:  # Machine Data -- section bits
            if len(data) > 12:
                section_bits = data[11]
                req.update_sections_from_aog(section_bits)
                logger.debug(f"AgIO sections byte=0x{section_bits:02X} -> {req.target_sections[:8]}")

                # Only send From Machine when HC5500 is alive
                if not req.boot_mode:
                    from_machine = build_from_machine(req.relay_lo, req.relay_hi)
                    sock.sendto(from_machine, broadcast)

        elif pgn == 0xFE:  # Steer Data -- speed (log only, not used)
            if len(data) > 6:
                spd = int.from_bytes(data[5:7], "little", signed=False) * 0.1
                logger.debug(f"AgIO speed={spd:.1f} km/h")

        # 0x64, 0xEB, others: silently ignored


def keyboard_loop(req: HCRequester):
    """Thread: keyboard input. X = exit."""
    logger.info("Keyboard: X = exit")
    while req.running:
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key in (b"x", b"X"):
                req.running = False
                logger.info("Exit requested")
                break
        time.sleep(0.05)


# ===================================================================
#  Main
# ===================================================================

def main():
    print("AOG-TUVR Bridge  (AgOpenGPS -> HC5500 section control)")
    print()

    # --- config ---
    config = load_config()
    saved_com = config.get("main", "com", fallback="0")
    comms_lost_zero = config.getboolean("main", "comms_lost_zero", fallback=True)

    # --- COM port selection ---
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
        timeout=0.02,
    )

    parser = PacketStreamParser()
    requester = HCRequester(ser)

    # --- start threads ---
    threads = [
        threading.Thread(target=udp_listener_loop, args=(requester, comms_lost_zero), daemon=True),
        threading.Thread(target=receiver_loop, args=(ser, parser, requester), daemon=True),
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
