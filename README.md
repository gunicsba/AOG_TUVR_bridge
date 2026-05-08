# AOG-TUVR Bridge

Bridge between **AgOpenGPS** and the **Trimble TUVR / HC5500** sprayer controller.

AgOpenGPS sends section states via UDP. This bridge translates them into
HC5500 serial commands (`S0C/6C`) so the controller opens and closes sections
in real time.

## Download

Grab the latest `AOG-TUVR.exe` from [Releases](../../releases).

## Requirements

- RS-232 connection to the HC5500 (USB-to-RS232 adapter + null modem cable)
- AgOpenGPS / AgIO broadcasting on UDP port 8888

For development:
- Python 3.8+
- [pyserial](https://pypi.org/project/pyserial/) (`pip install pyserial`)

## Usage

Run `AOG-TUVR.exe`. On first run you will be prompted to select a COM port.
The choice is saved to `config.ini` so subsequent runs connect automatically.

Press **X** to exit.

## How It Works

```
AgOpenGPS (AgIO)  UDP:8888        Keyboard (X=exit)
       |                                |
       v                                v
 [UDP listener]                  [keyboard thread]
       |                                |
       +------> shared state <----------+
               sections[13]
               agio_connected
                    |
                    v
          [HC5500 serial threads]
          TX: boot/run cycle every 0.2s
          RX: parse HC5500 responses
                    |
                    v
             HC5500 via RS-232
```

## AgOpenGPS PGNs

| PGN byte | Name | Direction | What the bridge does |
|----------|------|-----------|----------------------|
| `0xC8` | AgIO Hello | IN | Replies with Hello Machine PGN so the machine icon turns green |
| `0xEF` | Machine Data | IN | Byte 11 = 8-bit section mask, forwarded to HC5500 |
| `0xFE` | Steer Data | IN | Speed logged (not sent to HC5500) |
| `0x7B` | Hello Reply | OUT | Sent to AgIO on port 9999 in response to Hello |
| `0xED` | From Machine | OUT | Sent to AgIO on port 9999 with current relay state |

## HC5500 Serial Protocol

- **Baud:** 9600, 8N1
- **Framing:** `SOH HEADER STX PAYLOAD ETX CHECKSUM EOT`
- **Checksum:** XOR of ASCII bytes in HEADER + PAYLOAD, 2-char uppercase hex

### Run loop commands (5 Hz)

| Command | Purpose |
|---------|---------|
| `S0C 68,<rate>` | Set rate (l/ha / 10000) |
| `R0D 69` | Read back rate |
| `S0C 6C,<13 sections>` | Set section states (0/1 each) |
| `R0D 6B` | Read back section status |
| `R0D 6D` | Read back mode/status |

Boot mode sends `R0D 6A` at 1 Hz until HC5500 responds.

## config.ini

```ini
[main]
com = COM3
comms_lost_zero = 1
```

| Key | Description |
|-----|-------------|
| `com` | Saved COM port (`0` = prompt on startup) |
| `comms_lost_zero` | `1` = close all sections when AgIO stops responding (3s timeout) |

## Building

```bat
build.bat
```

Produces `AOG-TUVR.exe` via PyInstaller.
