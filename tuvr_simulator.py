"""Loopback TUVR VR-controller simulator (V2, minimal).

Pretends to be a TUVR-speaking sprayer controller.  Run it on one half of
a com0com null-modem pair and run AOG_TUVR_bridge.py on the other half to
validate the bridge end-to-end without any real hardware.

Usage:
    python tuvr_simulator.py --com COM10 --sections 8

Hotkeys while running:
    S  print current simulated state
    F  flip a random section and emit an unsolicited SECTION_STATE_RESP
    X  quit
"""
import argparse
import logging
import msvcrt
import random
import struct
import threading
import time
from typing import List, Optional

import serial

from tuvr_protocol import (
    FUNCTION,
    StreamParser,
    build_packet,
    pack_section_bits,
    unpack_section_bits,
)

BAUD = 38400

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s.%(msecs)03d] %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("tuvr-sim")


class SimulatorState:
    """Mutable state shared between threads."""

    def __init__(self, section_count: int) -> None:
        self.running = True
        self.section_count = section_count

        # Controller-side bookkeeping
        self.master_state = 0          # 0=OFF, 1=ON
        self.current_rate = 0          # mL/min (kept at 0 in V1)
        self.error_id = 0
        self.sections: List[bool] = [False] * section_count
        self.last_speed_mm_per_s = 0
        self.last_speed_source = 0

        # Serial write guard
        self.lock = threading.Lock()
        self.ser: Optional[serial.Serial] = None

    # ------------------------------------------------------------------
    def send(self, packet: bytes, label: str) -> None:
        if self.ser is None:
            return
        with self.lock:
            self.ser.write(packet)
            self.ser.flush()
        logger.info(f"TX >> {label} [{packet.hex()}]")

    def pretty_sections(self) -> str:
        return "".join("1" if b else "0" for b in self.sections)


# ===========================================================================
#  Packet senders (inline payload assembly)
# ===========================================================================

def send_status_resp(state: SimulatorState, txn_id: int) -> None:
    # payload: txn_id + current_mode + master_state + current_rate (4 LE) + error_id
    current_mode = 0x03
    payload = struct.pack("<BBBIB",
                          txn_id & 0xFF,
                          current_mode & 0xFF,
                          state.master_state & 0xFF,
                          state.current_rate & 0xFFFFFFFF,
                          state.error_id & 0xFF)
    state.send(build_packet(FUNCTION.STATUS, payload),
               f"STATUS_RESP txn={txn_id}")


def send_section_state_resp(state: SimulatorState, label_suffix: str = "") -> None:
    payload = pack_section_bits(state.sections)
    state.send(build_packet(FUNCTION.SECTION_STATE, payload),
               f"SECTION_STATE_RESP {state.pretty_sections()}{label_suffix}")


# ===========================================================================
#  Incoming packet dispatch
# ===========================================================================

def handle_packet(state: SimulatorState, id_byte: int,
                  function: int, payload: bytes) -> None:
    logger.info(
        f"RX << id=0x{id_byte:02X} fn=0x{function:02X} [{payload.hex()}]")

    if function == FUNCTION.STATUS:
        txn_id = payload[0] if payload else 0xFF
        send_status_resp(state, txn_id)

    elif function == FUNCTION.SECTION_STATE:
        if len(payload) == 0:
            # Pure request -- echo current state.
            send_section_state_resp(state, " (req)")
            return
        try:
            count, bits = unpack_section_bits(payload)
        except ValueError as e:
            logger.warning(f"SECTION_STATE_CMD malformed: {e}")
            return
        if count == 0xFFFF:
            logger.debug("SECTION_STATE_CMD data-change sentinel ignored")
            send_section_state_resp(state, " (echo)")
            return
        # Adopt the commanded sections (trim/pad to our configured count).
        new = [bool(bits[i]) if i < len(bits) else False
               for i in range(state.section_count)]
        if new != state.sections:
            state.sections = new
            logger.info(f"Sections commanded -> {state.pretty_sections()}")
        send_section_state_resp(state)

    elif function == FUNCTION.GPS_SPEED:
        if len(payload) >= 5:
            mm_per_s, source = struct.unpack_from("<IB", payload, 0)
            state.last_speed_mm_per_s = mm_per_s
            state.last_speed_source = source
            logger.debug(f"GPS_SPEED {mm_per_s}mm/s src={source}")
        # No reply (bridge ignores echoes anyway).

    else:
        logger.info(f"unknown function 0x{function:02X}, ignored")


# ===========================================================================
#  Threads
# ===========================================================================

def receiver_loop(state: SimulatorState) -> None:
    parser = StreamParser()
    while state.running:
        try:
            data = state.ser.read(256)
            if not data:
                continue
            for id_byte, function, payload in parser.feed(data):
                handle_packet(state, id_byte, function, payload)
        except Exception as e:
            logger.info(f"RX error: {e}")
            time.sleep(0.2)


def keyboard_loop(state: SimulatorState) -> None:
    logger.info("Keyboard: S=show state, F=flip section, X=quit")
    while state.running:
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key in (b"x", b"X"):
                state.running = False
                logger.info("Exit requested")
                break
            elif key in (b"s", b"S"):
                logger.info(
                    f"State: sections={state.pretty_sections()} "
                    f"master={state.master_state} "
                    f"last_speed={state.last_speed_mm_per_s}mm/s "
                    f"(src={state.last_speed_source})")
            elif key in (b"f", b"F"):
                i = random.randrange(state.section_count)
                state.sections[i] = not state.sections[i]
                logger.info(
                    f"Flipped section {i + 1} -> "
                    f"{state.pretty_sections()} "
                    f"(unsolicited SECTION_STATE_RESP)")
                send_section_state_resp(state, " (spontaneous)")
        time.sleep(0.05)


# ===========================================================================
#  Main
# ===========================================================================

def main() -> None:
    ap = argparse.ArgumentParser(
        description="Loopback TUVR VR-controller simulator")
    ap.add_argument("--com", required=True,
                    help="Serial port (e.g. COM10).")
    ap.add_argument("--sections", type=int, default=8,
                    help="Total simulated sections.")
    args = ap.parse_args()

    sections = max(1, min(255, args.sections))
    state = SimulatorState(section_count=sections)

    print(f"Simulator opening {args.com} @ {BAUD} baud")
    print(f"  sections={sections}")
    print()

    state.ser = serial.Serial(
        port=args.com,
        baudrate=BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
    )

    threads = [
        threading.Thread(target=receiver_loop, args=(state,), daemon=True),
        threading.Thread(target=keyboard_loop, args=(state,), daemon=True),
    ]
    for t in threads:
        t.start()

    try:
        while state.running:
            time.sleep(0.2)
    except KeyboardInterrupt:
        state.running = False
    finally:
        state.running = False
        time.sleep(0.3)
        if state.ser is not None:
            state.ser.close()
        logger.info("Serial port closed")


if __name__ == "__main__":
    main()
