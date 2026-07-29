#!/usr/bin/env python3
"""
crsf_test.py — Standalone CRSF decoder for testing an ELRS RX wired to the
Rpi4b's GPIO UART (before any ROS2 involvement).

Prints all 16 RC channel values live so you can wiggle sticks/switches on
the T8L and confirm you've found the right toggle channel index.

Usage:
    python3 crsf_test.py [serial_port]

Default port: /dev/ttyAMA0  (full PL011 UART on the 40-pin header, assuming
you disabled Bluetooth's claim on it per the setup steps below)
"""

import sys
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyAMA0"
BAUD = 420000  # CRSF standard baud rate

CRSF_SYNC = 0xC8
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16

# CRC8 (poly 0xD5) lookup table, as used by CRSF
def _make_crc8_table(poly=0xD5):
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        table.append(crc)
    return table

CRC8_TABLE = _make_crc8_table()


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = CRC8_TABLE[crc ^ b]
    return crc


def unpack_channels(payload: bytes):
    """Unpack 16 x 11-bit channel values from a 22-byte RC_CHANNELS_PACKED payload."""
    bits = 0
    bit_count = 0
    idx = 0
    channels = [0] * 16
    for byte in payload:
        bits |= byte << bit_count
        bit_count += 8
        while bit_count >= 11 and idx < 16:
            channels[idx] = bits & 0x7FF
            bits >>= 11
            bit_count -= 11
            idx += 1
    return channels


def main():
    print(f"Opening {PORT} @ {BAUD} baud...")
    ser = serial.Serial(PORT, BAUD, timeout=0.5)

    buf = bytearray()
    print("Listening for CRSF frames. Ctrl+C to stop.\n")

    try:
        while True:
            chunk = ser.read(64)
            if chunk:
                buf.extend(chunk)

            # Look for a sync byte and try to parse a frame
            while True:
                sync_idx = buf.find(bytes([CRSF_SYNC]))
                if sync_idx == -1:
                    buf.clear()
                    break
                if sync_idx > 0:
                    del buf[:sync_idx]

                if len(buf) < 3:
                    break  # need more data for length byte

                length = buf[1]  # bytes following: type + payload + crc
                frame_total = 2 + length  # sync + len byte + (type+payload+crc)

                if len(buf) < frame_total:
                    break  # wait for full frame

                frame = bytes(buf[:frame_total])
                del buf[:frame_total]

                frame_type = frame[2]
                payload = frame[3:-1]
                received_crc = frame[-1]
                calc_crc = crc8(frame[2:-1])  # type + payload

                if received_crc != calc_crc:
                    continue  # bad frame, skip

                if frame_type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED and len(payload) == 22:
                    channels = unpack_channels(payload)
                    line = " ".join(f"CH{i+1}:{v:4d}" for i, v in enumerate(channels))
                    print(f"\r{line}", end="", flush=True)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
