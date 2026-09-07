# UART1 Binary Debug Protocol And Python Utility

## Transport

- USART1 only: PA9 TX, PA10 RX.
- UART settings: 115200, 8N1, no flow control.
- RX in firmware is DMA-based:
  - receive 5-byte header first;
  - if packet is fixed-size, the 5 bytes are the full packet;
  - if packet is variable-size, validate header CRC, then arm DMA for `payload_size + 1` bytes.
- Endianness: little-endian for all 16-bit and 32-bit fields.
- CRC: CRC-8/ATM, polynomial `0x07`, init `0x00`, no xorout, table implementation in firmware.

## Common Header Byte

The first byte is shared by fixed and variable packets:

```text
bit 7     size_flag
bits 0-6  id
```

- `size_flag = 0`: fixed-size packet.
- `size_flag = 1`: variable-size packet.
- `id` is a 7-bit host transaction id. Firmware echoes it in command responses. Autonomous stream frames use `id = 0`.

## Fixed-Size Packet

Total size: 5 bytes.

```text
byte 0: id_flags  = (id & 0x7F)
byte 1: command   = uint8
byte 2: payload_l = uint16 payload, little-endian low byte
byte 3: payload_h = uint16 payload, little-endian high byte
byte 4: crc       = crc8(bytes 0..3)
```

Use fixed packets for small controls: ping, status request, stream start/stop with period, LED, debug LED, vibromotor.

## Variable-Size Packet

Header size: 5 bytes. Total size: `5 + payload_size + 1`.

```text
byte 0: id_flags   = 0x80 | (id & 0x7F)
byte 1: command    = uint8
byte 2: size_l     = uint16 payload_size, little-endian low byte
byte 3: size_h     = uint16 payload_size, little-endian high byte
byte 4: header_crc = crc8(bytes 0..3)
then payload_size bytes
last byte: payload_crc = crc8(payload bytes)
```

Firmware currently accepts payloads up to 256 bytes.

## Command IDs

| Command | Hex | Direction | Packet type | Payload |
| --- | ---: | --- | --- | --- |
| `PING` | `0x01` | host -> MCU | fixed | ignored |
| `STATUS` | `0x02` | host -> MCU | fixed | ignored |
| `STREAM_CONTROL` | `0x10` | host -> MCU | fixed or variable | start/stop stream |
| `LED_CONTROL` | `0x20` | host -> MCU | fixed or variable | action |
| `DBGLED_CONTROL` | `0x21` | host -> MCU | fixed or variable | action |
| `VIBRO_CONTROL` | `0x22` | host -> MCU | fixed or variable | action |
| `ACK` | `0x80` | MCU -> host | fixed | `(request_cmd << 8) | status` |
| `ERROR` | `0x81` | MCU -> host | fixed | `(request_cmd << 8) | error_code` |
| `STATUS_RESPONSE` | `0x82` | MCU -> host | variable | status payload |
| `STREAM_FRAME` | `0x83` | MCU -> host | variable | telemetry payload |

## Status And Error Codes

`ACK` low byte:

- `0x00`: OK.

`ERROR` low byte:

- `0x01`: bad CRC.
- `0x02`: bad size.
- `0x03`: RX packet overflow.
- `0x04`: unknown command.
- `0x05`: bad argument.
- `0x06`: DMA start/restart error.

## Control Payloads

Output action values:

- `0`: off.
- `1`: on.
- `2`: toggle.

For fixed output control packets:

```text
payload uint16:
  bits 0..7: action
  bits 8..15: reserved, send 0
```

For variable output control packets:

```text
payload[0] = action
```

PC13 LED is active-low because `Doc.md` marks the reversed output as `GPIO_PC13_WriteInverted`. PA1 debug LED and PC1 vibromotor are active-high in this debug interface.

## Stream Control Payload

Fixed packet:

```text
payload uint16:
  bit 15: enable
  bits 0..14: period_ms, 0 means default
```

Examples:

- Start 100 ms stream: payload `0x8064`.
- Stop stream: payload `0x0000`.

Variable packet:

```text
payload[0]   enable, 0 or 1
payload[1:3] period_ms uint16 little-endian, optional; 0 means default
```

Firmware clamps stream period to 50..5000 ms.

## STATUS_RESPONSE Payload

Command `0x82`, variable packet.

```text
u32 ms
u8  stream_enabled
u16 stream_period_ms
u8  led_logical
u8  led_gpio_raw
u8  dbgled_logical
u8  dbgled_gpio_raw
u8  vibro_logical
u8  vibro_gpio_raw
```

Total payload size: 13 bytes.

## STREAM_FRAME Payload

Command `0x83`, variable packet. Total payload size is currently 120 bytes.

Scale rules:

- Quaternion fields: signed int32, value = real value * `1_000_000`.
- Angle fields in degrees: signed int32, value = degrees * `1000`.
- Acceleration fields in m/s^2: signed int32, value = m/s^2 * `1000`.
- Gyro fields in rad/s: signed int32, value = rad/s * `1000`.
- ADS normalized fields: signed int32, value = normalized value * `1_000_000`.
- ADC converted fields are millivolts.

Payload layout:

```text
u32 ms
i32 imu_q0
i32 imu_q1
i32 imu_q2
i32 imu_q3
i32 imu_roll_deg
i32 imu_pitch_deg
i32 imu_yaw_deg
i32 imu_primary_deg
i32 imu_secondary_deg
i32 imu_comp_deg
i32 imu_primary_rate_dps
i32 imu_ax_mps2
i32 imu_ay_mps2
i32 imu_az_mps2
i32 imu_gx_rads
i32 imu_gy_rads
i32 imu_gz_rads
u32 imu_updates
u16 ads0_raw
u16 ads1_raw
u16 ads2_raw
u16 ads3_raw
i32 ads_norm0
i32 ads_norm1
i32 ads_raw_angle_deg
i32 ads_angle_deg
i32 ads_filtered_deg
u8  ads_angle_valid
u16 adc8_raw
u16 adc8_mv
u8  adc8_ok
u16 adc9_raw
u16 adc9_mv
u8  adc9_ok
u16 adc13_raw
u16 adc13_mv
u8  adc13_ok
```

ADC conversion used by firmware:

- ADC8 PB0: `adc8_mv = raw / 4096 * 2 * 3300`.
- ADC9 PB1: `adc9_mv = raw / 4096 * 2 * 3300`.
- ADC13 PC3: `adc13_mv = raw / 4096 * 4.091 * 3300`, implemented as integer full-scale about 13500 mV.

The stream reports the latest filtered IMU state owned by `ImuController`; the debug UART path does not force a direct MPU read.

## Python Utility Requirements

Use Python 3.10+ and `pyserial`.

```bash
python -m pip install pyserial
```

Suggested commands:

- `ping`
- `status`
- `stream --period 100`
- `stop`
- `led on|off|toggle`
- `dbgled on|off|toggle`
- `vibro on|off|toggle`
- `log --period 100 --csv file.csv`

Suggested CLI args:

- `--port COMx`
- `--baud 115200`
- `--timeout 1.0`
- `--period 100`
- `--csv out.csv`

## Python CRC And Framing Sketch

```python
def build_crc8_table(poly: int = 0x07) -> list[int]:
    table = []
    for index in range(256):
        crc = index
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        table.append(crc)
    return table

CRC8_TABLE = build_crc8_table()

def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc = CRC8_TABLE[crc ^ byte]
    return crc

def fixed_packet(packet_id: int, command: int, payload: int = 0) -> bytes:
    frame = bytes([
        packet_id & 0x7F,
        command & 0xFF,
        payload & 0xFF,
        (payload >> 8) & 0xFF,
    ])
    return frame + bytes([crc8(frame)])

def variable_packet(packet_id: int, command: int, payload: bytes) -> bytes:
    size = len(payload)
    header = bytes([
        0x80 | (packet_id & 0x7F),
        command & 0xFF,
        size & 0xFF,
        (size >> 8) & 0xFF,
    ])
    return header + bytes([crc8(header)]) + payload + bytes([crc8(payload)])
```

## Python RX Sketch

```python
def read_packet(ser):
    header = ser.read(5)
    if len(header) != 5:
        raise TimeoutError("header timeout")

    packet_id = header[0] & 0x7F
    variable = bool(header[0] & 0x80)
    command = header[1]

    if not variable:
        if crc8(header[:4]) != header[4]:
            raise ValueError("fixed crc")
        payload = header[2] | (header[3] << 8)
        return packet_id, command, payload

    size = header[2] | (header[3] << 8)
    if crc8(header[:4]) != header[4]:
        raise ValueError("header crc")

    body = ser.read(size + 1)
    if len(body) != size + 1:
        raise TimeoutError("payload timeout")
    payload = body[:-1]
    if crc8(payload) != body[-1]:
        raise ValueError("payload crc")
    return packet_id, command, payload
```

## Utility Behavior Checklist

1. Open serial port.
2. Send `PING` fixed packet and require `ACK` for command `0x01`.
3. Send `STATUS` fixed packet and parse `STATUS_RESPONSE`.
4. For logging, send `STREAM_CONTROL` with enable and period.
5. Read packets in a loop; parse `STREAM_FRAME` payload by the exact layout above.
6. On Ctrl+C send fixed `STREAM_CONTROL` payload `0x0000`.
7. Treat `ERROR` packets as visible errors with command and error code.
