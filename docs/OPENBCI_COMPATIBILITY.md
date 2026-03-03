# ESP32 + Dual ADS1299-4 OpenBCI Compatibility Notes

## What this firmware emulates

This project now emulates an **8-channel Cyton-style serial board** for OpenBCI GUI:

- 8 EEG channels (4 from ADS1 + 4 from ADS2)
- Cyton packet framing: `0xA0 ... 0xC0`
- 33-byte packet length
- 24-bit signed channel payloads (big-endian per channel)
- 3 aux channels (currently zeros)

## OpenBCI GUI transport expectations

### 1) Serial (recommended for this firmware)

OpenBCI GUI Cyton serial mode expects a USB serial port. This is exactly what this firmware targets.

- Use your USB-UART connection to the ESP32
- Baud configured in firmware: `115200`
- GUI should be set to **Cyton** + **Serial**

### 2) Bluetooth

OpenBCI GUI Bluetooth paths are primarily for Ganglion/BLE workflows, not generic Cyton-over-BLE serial emulation.

If you want BLE/Wi-Fi instead of USB serial, you have two robust options:

1. Keep this firmware as-is and bridge data to LSL/UDP via a host app.
2. Implement BrainFlow-compatible BLE/Wi-Fi transport profile expected by GUI modes.

For fastest stability, use USB serial first.

## Implemented command compatibility

Implemented in firmware command parser:

- `v` board/version info text
- `b` start streaming
- `s` stop streaming
- `d` apply default ADS settings
- `c` announce 8-channel mode (`daisy removed`)
- `C` reject daisy request (`no daisy to attach`)
- `~0..~6` data-rate selection (ADS1299 DR bits)
- `x...X` per-channel ADS settings command
- `z...Z` impedance-mode style command (basic support)
- legacy channel toggles: `1..8` off, `!@#$%^&*` on

## ADS1299 defaults applied (Cyton-like)

On startup and `d` command:

- Data rate = `250 SPS` (`CONFIG1[2:0] = 110`)
- Gain = `x24`
- Input type = `NORMAL`
- Bias include = ON
- SRB2 = ON
- SRB1 = OFF

## Channel mapping

- Channel 1..4 -> ADS1 CH1..CH4
- Channel 5..8 -> ADS2 CH1..CH4

## Packet format detail

Each sample packet (33 bytes):

1. Byte 0: `0xA0`
2. Byte 1: sample counter (`0..255`, wraps)
3. Bytes 2..25: 8 channels × 3 bytes signed 24-bit
4. Bytes 26..31: aux payload (6 bytes, currently zero)
5. Byte 32: `0xC0`

## Practical setup checklist

1. Flash firmware and open serial monitor once to verify startup text.
2. In OpenBCI GUI select:
   - Data source: Cyton (live)
   - Transfer protocol: Serial
   - Correct COM port
   - 8 channels
3. Start session, then stream.
4. Use GUI Hardware Settings to change gain/input/bias/SRB per channel (handled through `x...X`).

## EEG quality reminders

- Keep electrode impedance low and balanced.
- Use proper reference electrode and bias electrode contact.
- Keep cables short, shielded, and away from mains noise.
- Use battery/isolated power for low-noise and safety.
