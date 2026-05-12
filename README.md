# AOG_TUVR_bridge

AgOpenGPS <-> TUVR variable-rate controller bridge.

Reads section and speed PGNs from AgIO over UDP, drives a binary serial
link to a TUVR-speaking rate controller at 38400 8-N-1, and feeds the
controller's reported section state back into AgIO so the rate
controller UI mirrors reality.

## What it does

- Listens on UDP port `8888` for AgOpenGPS PGNs (`0xC8` Hello, `0xEF`
  Machine Data, `0xFE` Steer Data).
- Broadcasts feedback PGNs back (`0xEA` Section Data, `0xED` From
  Machine, Hello reply, switch-box PGN `32618`).
- Talks a minimal TSIP-style framed serial protocol to the controller
  with three wire messages:
  - `STATUS` (`0x00`) - heartbeat probe, sent at `status_hz`.
  - `SECTION_STATE` (`0x06`) - bit-packed section command out, echo
    back in.
  - `GPS_SPEED` (`0x81`) - speed in mm/s with a validity flag.
- Also ships a loopback simulator
  ([tuvr_simulator.py](tuvr_simulator.py)) that pretends to be the
  controller on the other half of a com0com null-modem pair.

## Files

| File | Purpose |
|---|---|
| [AOG_TUVR_bridge.py](AOG_TUVR_bridge.py) | The bridge. Threads: UDP listener, serial receiver, periodic scheduler, keyboard. |
| [tuvr_protocol.py](tuvr_protocol.py) | Framing, checksum, packet builders, `StreamParser`, section bit-packing. |
| [tuvr_simulator.py](tuvr_simulator.py) | Loopback simulator for hardware-free testing. |
| [build.bat](build.bat) | PyInstaller one-file build -> `AOG-TUVR.exe`. |
| [startup.bat](startup.bat) | Convenience launcher. |
| [config.ini](config.ini) | Auto-created on first run (see below). |
| [icon.ico](icon.ico) | App icon for the frozen exe. |

## Requirements

- Windows (uses `msvcrt` for keyboard polling).
- Python 3.8+.
- `pyserial`.
- `pyinstaller` (only for building the exe).

Install:

```
pip install pyserial pyinstaller
```

## Running from source

```
python AOG_TUVR_bridge.py
```

First run picks a COM port interactively and writes `config.ini`
next to the script. Press `X` in the console window to exit cleanly.

## Building the exe

```
build.bat
```

Produces `AOG-TUVR.exe` in the project root.

## config.ini

Auto-generated on first run. Defaults:

```ini
[main]
com = 0
comms_lost_zero = 1
sections = 8
sct_hz = 5
spd_hz = 5
status_hz = 1
subnet = 255.255.255.255
```

| Key | Meaning |
|---|---|
| `com` | Saved COM port (e.g. `COM7`). `0` = ask on startup. |
| `comms_lost_zero` | On AgIO timeout, force all sections off and speed to zero. |
| `sections` | Total sections sent in `SECTION_STATE_CMD`. Clamped to 1..16. |
| `sct_hz` | `SECTION_STATE_CMD` send rate. |
| `spd_hz` | `GPS_SPEED_CMD` send rate. |
| `status_hz` | `STATUS_REQ` heartbeat / probe rate. |
| `subnet` | Broadcast address for AgIO PGNs. |

## Serial settings

`38400` baud, 8-N-1, no flow control. Typically wired as a null-modem
(crossover) cable between the PC and the controller — see
[nullmodemkabel.jpg](nullmodemkabel.jpg).

## State machine

- **DISCONNECTED** - no valid controller frame seen, or last one was
  more than `MACHINE_TIMEOUT_S = 5 s` ago. `STATUS_REQ` still fires at
  `status_hz` as a probe.
- **READY** - controller replied at least once and AgIO is not
  connected yet.
- **RUNNING** - READY plus AgIO is sending PGNs. `SECTION_STATE_CMD`
  and `GPS_SPEED_CMD` are streamed at their configured rates.

Any incoming valid frame bumps DISCONNECTED -> READY. AgIO timeout
drops RUNNING -> READY.

## Testing without hardware

Set up a com0com null-modem pair (e.g. `COM10` <-> `COM11`) and in two
consoles:

```
python tuvr_simulator.py --com COM11 --sections 8
python AOG_TUVR_bridge.py
```

Pick `COM10` when the bridge asks. Hotkeys in the simulator:

| Key | Action |
|---|---|
| `S` | Print current simulated state. |
| `F` | Flip a random section and emit an unsolicited `SECTION_STATE_RESP`. |
| `X` | Quit. |

## Wire format (summary)

```
0x10 0x8E 0xAA  <function>  <payload...>  <ck_lo> <ck_hi>  0x10 0x03
```

Every byte between the leading `0x10` and the trailing `0x10 0x03` is
escaped by doubling any `0x10` to `0x10 0x10`; the stream parser
collapses them back on receive. Checksum is a 16-bit unsigned sum of
`0xAA + function + payload`, transmitted little-endian.

Only three functions are used on the wire:

| Name | Value | Direction | Payload |
|---|---|---|---|
| `STATUS` | `0x00` | req out, reply in | 1-byte `txn_id` (we always send `0xFF`) |
| `SECTION_STATE` | `0x06` | both ways | empty = read, or `reserved + count(LE16) + bit-packed bits` |
| `GPS_SPEED` | `0x81` | out | `uint32 mm/s (LE) + 1-byte source` |

Any other function received from the controller is logged and ignored.

## License / legal

This bridge is a clean-room implementation for AgOpenGPS. It does not
include, redistribute or quote any third-party protocol specification.
