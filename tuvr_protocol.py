"""TUVR binary serial protocol

Shared by AOG_TUVR_bridge.py (display side) and tuvr_simulator.py
(controller side).

Wire format
-----------
Every outbound frame has the folllowing shape:

    0x10 0x8E 0xAA  <function>  <payload...>  <ck_lo> <ck_hi>  0x10 0x03

All bytes between the leading 0x10 and the trailing 0x10 0x03 are
escaped by doubling every 0x10 -> 0x10 0x10 on the wire.  The stream
parser collapses the doubled bytes back to a single 0x10 on receive.

Checksum: unsigned 16-bit sum of (0xAA + function + payload),
transmitted little-endian (lo byte first).

Only three wire functions are used:

    STATUS         0x00   (heartbeat; payload = 1-byte txn_id, hardcoded 0xFF)
    SECTION_STATE  0x06   (request with empty payload, or bit-packed command)
    GPS_SPEED      0x81   (uint32 mm/s + 1-byte source)
"""
from __future__ import annotations

import struct
from enum import IntEnum
from typing import Iterator, List, Tuple

# ---------------------------------------------------------------------------
#  Framing constants (hardcoded)
# ---------------------------------------------------------------------------
DLE = 0x10
ETX = 0x03

HEADER = bytes([0x10, 0x8E, 0xAA])   # what we send
TAIL = bytes([0x10, 0x03])

# Hardcoded transaction id used in every STATUS_REQ we emit.
TXN_ID = 0xFF

# Kept only so the parser/log lines can tag "ours vs theirs".
CMD_ID = 0x8E   # display-to-controller
RESP_ID = 0x8F  # controller-to-display


class FUNCTION(IntEnum):
    STATUS = 0x00
    SECTION_STATE = 0x06
    GPS_SPEED = 0x81


# ---------------------------------------------------------------------------
#  Checksum + packet assembly
# ---------------------------------------------------------------------------

def sum_checksum(body: bytes) -> Tuple[int, int]:
    """16-bit unsigned sum of *body*. Returns (lo, hi)."""
    s = sum(body) & 0xFFFF
    return s & 0xFF, (s >> 8) & 0xFF


def build_packet(function: int, payload: bytes = b"") -> bytes:
    """Assemble a complete, escaped frame.

    Mental model: HEADER + (payload bytes with 0x10 doubled) + checksum + TAIL.
    In practice we also have to escape the function byte and the checksum
    bytes, because they can legitimately be 0x10 and that would break
    framing; the escape is applied to everything between the leading 0x10
    and the trailing 0x10 0x03.
    """
    body = bytes([0xAA, function & 0xFF]) + payload
    lo, hi = sum_checksum(body)
    middle = bytes([0x8E]) + body + bytes([lo, hi])
    escaped = middle.replace(b"\x10", b"\x10\x10")
    return bytes([0x10]) + escaped + TAIL


# ---------------------------------------------------------------------------
#  Packet builders we keep
# ---------------------------------------------------------------------------

def build_status_req() -> bytes:
    """STATUS heartbeat with the hardcoded txn id."""
    return build_packet(FUNCTION.STATUS, bytes([TXN_ID]))


def build_section_state_req() -> bytes:
    """SECTION_STATE read-back (empty payload)."""
    return build_packet(FUNCTION.SECTION_STATE, b"")


def build_section_state_cmd(section_bits: List[bool]) -> bytes:
    """SECTION_STATE command with the bit-packed section states."""
    return build_packet(FUNCTION.SECTION_STATE, pack_section_bits(section_bits))


def build_gps_speed_cmd(mm_per_s: int, source: int) -> bytes:
    """GPS_SPEED: uint32 mm/s little-endian + 1-byte source."""
    payload = struct.pack("<IB",
                          mm_per_s & 0xFFFFFFFF,
                          source & 0xFF)
    return build_packet(FUNCTION.GPS_SPEED, payload)


# ---------------------------------------------------------------------------
#  Stream parser
# ---------------------------------------------------------------------------

class StreamParser:
    """Feed raw serial bytes, receive complete frames.

    Each call to :meth:`feed` yields zero or more tuples:

        (id_byte, function, payload_bytes)

    Malformed frames (bad checksum, wrong sub-id, too short) are dropped
    silently; the parser resynchronises on the next 0x10.
    """

    _IDLE = 0
    _ID = 1
    _BODY = 2
    _DLE_IN_BODY = 3

    def __init__(self) -> None:
        self._state = self._IDLE
        self._id = 0
        self._buf = bytearray()

    def feed(self, data: bytes) -> Iterator[Tuple[int, int, bytes]]:
        for b in data:
            if self._state == self._IDLE:
                if b == DLE:
                    self._state = self._ID

            elif self._state == self._ID:
                if b == DLE:
                    # Double DLE at frame start = stream noise, stay in ID.
                    continue
                self._id = b
                self._buf = bytearray()
                self._state = self._BODY

            elif self._state == self._BODY:
                if b == DLE:
                    self._state = self._DLE_IN_BODY
                else:
                    self._buf.append(b)

            elif self._state == self._DLE_IN_BODY:
                if b == DLE:
                    # Escaped 0x10 data byte.
                    self._buf.append(DLE)
                    self._state = self._BODY
                elif b == ETX:
                    frame = bytes(self._buf)
                    self._state = self._IDLE
                    result = self._finalize(self._id, frame)
                    if result is not None:
                        yield result
                else:
                    # Lone DLE followed by unexpected byte -- drop and resync.
                    self._state = self._IDLE

    def _finalize(self, id_byte: int, frame: bytes):
        # frame = 0xAA + function + payload + ck_lo + ck_hi
        if len(frame) < 4:
            return None
        body = frame[:-2]
        ck_lo = frame[-2]
        ck_hi = frame[-1]
        exp_lo, exp_hi = sum_checksum(body)
        if (ck_lo, ck_hi) != (exp_lo, exp_hi):
            return None
        if body[0] != 0xAA:
            return None
        function = body[1]
        payload = bytes(body[2:])
        return id_byte, function, payload


# ===========================================================================
#  SECTION_STATE bit-packing
# ===========================================================================
#
# Payload layout for SECTION_STATE command and read-back:
#
#     reserved : 1 byte  (always 0x80)
#     count    : uint16 little-endian
#                0xFFFF   = "data-change only, peer should use last count"
#                0x0000   = no sections follow
#                1..255   = number of sections packed in the bytes that follow
#     sections : ceil(count/8) bytes, little-endian bit-packed,
#                section i (1-based) is bit ((i-1) & 7) of byte ((i-1) >> 3).
#                Unused upper bits of the final byte are zero.
# ===========================================================================

SECTION_RESERVED = 0x80
SECTION_COUNT_CHANGE = 0xFFFF


def pack_section_bits(section_bits: List[bool]) -> bytes:
    """Bit-pack a list of bool/int section states (section 1 first)."""
    n = len(section_bits)
    if n == 0:
        return bytes([SECTION_RESERVED, 0, 0])
    nbytes = (n + 7) // 8
    out = bytearray(nbytes)
    for i, on in enumerate(section_bits):
        if on:
            out[i >> 3] |= 1 << (i & 7)
    return bytes([SECTION_RESERVED]) + struct.pack("<H", n) + bytes(out)


def unpack_section_bits(payload: bytes) -> Tuple[int, List[bool]]:
    """Return ``(count, bits)`` from a SECTION_STATE payload.

    If *count* is 0xFFFF the data-change-only sentinel is returned as-is;
    *bits* will be ``[]`` in that case and the caller should fall back
    to the last known count.
    """
    if len(payload) < 3:
        raise ValueError("SECTION_STATE payload too short")
    # payload[0] = reserved; ignored on receive
    count = struct.unpack_from("<H", payload, 1)[0]
    if count == SECTION_COUNT_CHANGE:
        return count, []
    need = (count + 7) // 8
    if len(payload) < 3 + need:
        raise ValueError("SECTION_STATE payload shorter than count")
    bits: List[bool] = []
    for i in range(count):
        byte = payload[3 + (i >> 3)]
        bits.append(bool((byte >> (i & 7)) & 1))
    return count, bits
