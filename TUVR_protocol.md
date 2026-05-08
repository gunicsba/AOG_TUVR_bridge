# TUVR Serial Protocol (Trimble ↔ HC5500)

Reverse engineering notes for the serial protocol between a Trimble controller and an HC5500 sprayer controller.

---

## Status

At this point, the following are understood well:

- Physical layer: **RS232**, **9600 baud**, 8N1
- Message framing
- Checksum algorithm
- Boot vs run state machine
- Rate control via `S0C 68`
- Request/response flow for `6A`, `69`, `6B`, `6D`
- Section state write/report via `S0C 6C`
- Continuous run loop behavior at about **5 Hz**
- **10k resistor required for sniffing**

---

## 1. Physical layer

- Interface: **RS232**
- Speed: **9600 baud**
- Format: **8 data bits, no parity, 1 stop bit**

### Critical sniffing note

**A 10k resistor is required for sniffing.**

Without it:
- the line may become unstable
- Trimble may stop transmitting
- communication may partially freeze

This was a critical practical discovery.

---

## 2. Message framing

Messages are framed like this:

```text
0x01 <HEADER> 0x02 <PAYLOAD> 0x03 <CHECKSUM_ASCII_HEX> 0x04
```

### Byte meanings

| Byte | Meaning |
|---|---|
| `0x01` | SOH |
| ASCII | Header (`R0D`, `A0D`, `S0C`, `V0C`, `N0C`) |
| `0x02` | STX |
| ASCII | Payload |
| `0x03` | ETX |
| ASCII | 2-character checksum |
| `0x04` | EOT |

---

## 3. Header types

### `R0D`
Read / request.

Examples:
- `R0D 6A`
- `R0D 69`
- `R0D 6B`
- `R0D 6D`

### `A0D`
Answer / response.

Examples:
- `A0D 6A,...`
- `A0D 69,...`
- `A0D 6B,...`
- `A0D 6D,...`

### `S0C`
Set / write / report.

Known important records:
- `S0C 68,<value>` → set rate
- `S0C 6C,<13 section values>` → set section states

### `V0C`
Value / report family.

Known:
- `V0C 68,<value>`

### `N0C`
Another report family.

Known:
- `N0C 6C,<13 section values>`

---

## 4. Checksum

### Algorithm

Checksum is the XOR of the ASCII bytes of:

```text
HEADER + PAYLOAD
```

The following bytes are **not** included:
- `0x01`
- `0x02`
- `0x03`
- `0x04`

The checksum is then sent as:
- **2 uppercase ASCII hex characters**

### Formula

```text
checksum = XOR(ASCII(HEADER + PAYLOAD))
```

---

## 5. Checksum examples

### Example: `R0D 6A`

Checksum input:

```text
R0D6A
```

Checksum:

```text
51
```

Full frame:

```text
01 52 30 44 02 36 41 03 35 31 04
```

---

### Example: `S0C 68,0.0200`

Checksum input:

```text
S0C68,0.0200
```

Checksum:

```text
1E
```

Full frame:

```text
01 53 30 43 02 36 38 2C 30 2E 30 32 30 30 03 31 45 04
```

---

### Example: `S0C 68,0.0100`

Checksum input:

```text
S0C68,0.0100
```

Checksum:

```text
1D
```

Full frame:

```text
01 53 30 43 02 36 38 2C 30 2E 30 31 30 30 03 31 44 04
```

---

### Example: `S0C 68,0.0150`

Checksum input:

```text
S0C68,0.0150
```

Checksum:

```text
18
```

Full frame:

```text
01 53 30 43 02 36 38 2C 30 2E 30 31 35 30 03 31 38 04
```

---

### Example: `S0C 68,0.0250`

Checksum input:

```text
S0C68,0.0250
```

Checksum:

```text
1B
```

Full frame:

```text
01 53 30 43 02 36 38 2C 30 2E 30 32 35 30 03 31 42 04
```

---

### Example: `S0C 6C,1,1,1,1,1,1,1,1,1,1,1,1,1`

Checksum input:

```text
S0C6C,1,1,1,1,1,1,1,1,1,1,1,1,1
```

Checksum:

```text
48
```

Full frame:

```text
01 53 30 43 02 36 43 2C 31 2C 31 2C 31 2C 31 2C 31 2C 31 2C 31 2C 31 2C 31 2C 31 2C 31 2C 31 2C 31 03 34 38 04
```

This matches the sniffed frame:

```text
S0C6C,1,1,1,1,1,1,1,1,1,1,1,1,148
```

---

### Example: `S0C 6C,0,0,0,0,0,0,0,0,0,0,0,0,0`

Checksum input:

```text
S0C6C,0,0,0,0,0,0,0,0,0,0,0,0,0
```

Checksum:

```text
49
```

Full frame:

```text
01 53 30 43 02 36 43 2C 30 2C 30 2C 30 2C 30 2C 30 2C 30 2C 30 2C 30 2C 30 2C 30 2C 30 2C 30 2C 30 03 34 39 04
```

---

## 6. Why `0.1200` and `0.0120` can produce the same checksum

Examples:

- `S0C 68,0.1200` → checksum `1F`
- `S0C 68,0.0120` → checksum `1F`

This is expected for XOR:
- XOR depends on byte values
- certain byte order changes do not change the final XOR if the same set of bytes is used

---

## 7. Python helper functions

These are the important utility functions for actual use.

### Checksum calculator

```python
def xor_checksum_ascii(header: str, payload: str) -> str:
    x = 0
    for ch in (header + payload):
        x ^= ord(ch)
    return f"{x:02X}"
```

### Frame builder

```python
SOH = 0x01
STX = 0x02
ETX = 0x03
EOT = 0x04

def build_packet(header: str, payload: str) -> bytes:
    cs = xor_checksum_ascii(header, payload)
    return (
        bytes([SOH]) +
        header.encode("ascii") +
        bytes([STX]) +
        payload.encode("ascii") +
        bytes([ETX]) +
        cs.encode("ascii") +
        bytes([EOT])
    )
```

### Generic send function

```python
def send_packet(ser, header: str, payload: str):
    packet = build_packet(header, payload)
    ser.write(packet)
    ser.flush()
```

---

## 8. Practical Python examples that actually work

### Example: send `R0D 6A`

```python
send_packet(ser, "R0D", "6A")
```

This generates and sends:

```text
01 52 30 44 02 36 41 03 35 31 04
```

---

### Example: set 200 l/ha

200 l/ha = `0.0200`

```python
send_packet(ser, "S0C", "68,0.0200")
```

This generates and sends:

```text
01 53 30 43 02 36 38 2C 30 2E 30 32 30 30 03 31 45 04
```

---

### Example: set 250 l/ha

250 l/ha = `0.0250`

```python
send_packet(ser, "S0C", "68,0.0250")
```

This generates and sends:

```text
01 53 30 43 02 36 38 2C 30 2E 30 32 35 30 03 31 42 04
```

---

### Example: set all 13 sections ON

```python
sections = "1,1,1,1,1,1,1,1,1,1,1,1,1"
send_packet(ser, "S0C", f"6C,{sections}")
```

This generates and sends:

```text
01 53 30 43 02 36 43 2C 31 2C 31 2C 31 2C 31 2C 31 2C 31 2C 31 2C 31 2C 31 2C 31 2C 31 2C 31 2C 31 03 34 38 04
```

---

### Example: set all 13 sections OFF

```python
sections = "0,0,0,0,0,0,0,0,0,0,0,0,0"
send_packet(ser, "S0C", f"6C,{sections}")
```

This generates and sends:

```text
01 53 30 43 02 36 43 2C 30 2C 30 2C 30 2C 30 2C 30 2C 30 2C 30 2C 30 2C 30 2C 30 2C 30 2C 30 2C 30 03 34 39 04
```

---

## 9. Example serial program

This is a minimal working example showing how to open a serial port, build the checksum automatically, and send frames.

```python
import serial

SOH = 0x01
STX = 0x02
ETX = 0x03
EOT = 0x04

def xor_checksum_ascii(header: str, payload: str) -> str:
    x = 0
    for ch in (header + payload):
        x ^= ord(ch)
    return f"{x:02X}"

def build_packet(header: str, payload: str) -> bytes:
    cs = xor_checksum_ascii(header, payload)
    return (
        bytes([SOH]) +
        header.encode("ascii") +
        bytes([STX]) +
        payload.encode("ascii") +
        bytes([ETX]) +
        cs.encode("ascii") +
        bytes([EOT])
    )

def send_packet(ser, header: str, payload: str):
    packet = build_packet(header, payload)
    ser.write(packet)
    ser.flush()
    print(f"SENT: {packet.hex(' ').upper()}")

ser = serial.Serial("COM48", 9600, timeout=0.05)

# Request configuration
send_packet(ser, "R0D", "6A")

# Set 200 l/ha
send_packet(ser, "S0C", "68,0.0200")

# Set all sections on
send_packet(ser, "S0C", "6C,1,1,1,1,1,1,1,1,1,1,1,1,1")

ser.close()
```

---

## 10. Rate scaling

Rate is not sent directly in l/ha.

Conversion is:

```text
scaled = l_ha / 10000
```

### Examples

| l/ha | transmitted |
|---:|---|
| 100 | `0.0100` |
| 120 | `0.0120` |
| 150 | `0.0150` |
| 200 | `0.0200` |
| 250 | `0.0250` |
| 1200 | `0.1200` |
| 1500 | `0.1500` |

---

## 11. Record meanings

### `6A`
Configuration / geometry.

Example:

```text
A0D 6A,24.00,08,04,06,06,08,08,06,06,04,12,12,12,12,12,A
```

Interpretation:
- `24.00` = boom width in meters
- `08` = real section count
- `04,06,06,08,08,06,06,04` = actual physical section structure
- remaining `12`s = filler to reach 13 protocol section slots

---

### `68`
Rate set/report record.

Known good write:
```text
S0C 68,<scaled rate>
```

---

### `69`
Rate readback / target rate response.

Observed:
```text
A0D 69,0.01500,0.00000,A
```

---

### `6B`
Section-related response record.

Observed:
```text
A0D 6B,0,0,0,0,0,0,0,0,0,0,0,0,0,A
```

Currently treated as readback / response, not primary write.

---

### `6C`
Section state write/report.

Examples:
```text
S0C 6C,0,0,0,0,0,0,0,0,0,0,0,0,0
S0C 6C,1,1,1,1,1,1,1,1,1,1,1,1,1
```

Always 13 values.

---

### `6D`
Mode / state / flag record.

Observed:
```text
A0D 6D,L,00,A
```

Exact meaning still not fully decoded.

---

## 12. Boot sequence

During boot, Trimble repeatedly requests:

```text
R0D 6A
```

HC responds with configuration and related records.

Practical state machine:

- **BOOT mode**
  - repeatedly send `R0D 6A`
  - wait for valid HC response
- **RUN mode**
  - enter after first valid response
- **fallback to BOOT**
  - if HC stops responding for too long

---

## 13. Run loop

The effective Trimble-side run sequence is:

```text
S0C 68
R0D 69
S0C 6C
R0D 6B
R0D 6D
```

### Practical frequency

About:

```text
5 Hz
```

which means:

```python
RUN_PERIOD_S = 0.2
```

---

## 14. Minimal implementation pattern

```python
def send_run_cycle(self):
    sec = ",".join(str(x) for x in self.target_sections)
    scaled = self.scaled_rate()

    self.send_packet("S0C", f"68,{scaled}")
    time.sleep(REQUEST_GAP_S)

    self.send_packet("R0D", "69")
    time.sleep(REQUEST_GAP_S)

    self.send_packet("S0C", f"6C,{sec}")
    time.sleep(REQUEST_GAP_S)

    self.send_packet("R0D", "6B")
    time.sleep(REQUEST_GAP_S)

    self.send_packet("R0D", "6D")
```

---

## 15. Main practical findings

### What worked
- 10k resistor for sniffing
- XOR checksum
- BOOT → RUN switching
- `S0C 68` for dose write
- `S0C 6C` as section write/report
- 5 Hz continuous run loop

### What failed
- overlong boot transmission
- trying to write sections via `A0D 6B,...,A`
- wrong run ordering
- assuming `69` itself is the primary write record

---

## 16. Final engineering takeaway

To emulate Trimble successfully, it is not enough to send valid packets.

You must also:
- send the correct **record family**
- send the correct **checksum**
- send records in the correct **order**
- maintain the correct **state machine**
- keep the protocol alive continuously at about **5 Hz**
