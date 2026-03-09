#!/usr/bin/env python3
import sys
import time
from collections import deque
from typing import Deque
from elrs.telemetry import _parse_attitude, _parse_battery, _parse_gps, _parse_linkstats, _parse_vario, _parse_baro_alt, frames_from_bytes, FTYPE_NAMES
import serial

# ====== Jetson UART port ======
PORT = "/dev/ttyTHS1"     
BAUD = 921600             
RATE_HZ = 500             # from lua script

ADDRS_START = {0xC8, 0xEA, 0xEE}       # FC, RadioTX, CRSF TX

FT_LINKSTAT = 0x14
FT_BATTERY  = 0x08
FT_GPS      = 0x02
FT_ATTITUDE = 0x1E
FT_VARIO    = 0x07
FT_BARO_ALT = 0x09

# Map frame-type → decoder
_DECODERS = {
    FT_LINKSTAT: _parse_linkstats,
    FT_BATTERY:  _parse_battery,
    FT_GPS:      _parse_gps,
    FT_ATTITUDE: _parse_attitude,
    FT_VARIO:    _parse_vario,
    FT_BARO_ALT: _parse_baro_alt,
}

FTYPE_NAMES = {
    FT_LINKSTAT: "linkstats",
    FT_BATTERY:  "battery",
    FT_GPS:      "gps",
    FT_ATTITUDE: "attitude",
    FT_VARIO:    "vario",
    FT_BARO_ALT: "baro_alt",
}
IGNORE_TYPES = {0x16, 0x21, 0x3A}
# ====== CRSF helpers ======
def crc8_dvb_s2(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def us_to_crsf11(us: int) -> int:
    """
    Map 1000..2000 us -> CRSF 11-bit value.
    Common mapping used in many implementations:
      crsf = (us - 1500) * 8/5 + 992
    """
    if us < 1000: us = 1000
    if us > 2000: us = 2000
    v = int(round((us - 1500) * 8 / 5 + 992))
    return max(0, min(2047, v))

def pack_channels_16x11(ch_us):
    """
    Pack 16 channels (1000..2000us) into 22-byte CRSF payload (type 0x16).
    Packing is little-endian bitstream per CRSF spec.
    """
    if len(ch_us) != 16:
        raise ValueError("Need exactly 16 channels")

    ch11 = [us_to_crsf11(v) for v in ch_us]

    out = bytearray()
    bits = 0
    bitlen = 0
    for x in ch11:
        bits |= (x << bitlen)
        bitlen += 11
        while bitlen >= 8:
            out.append(bits & 0xFF)
            bits >>= 8
            bitlen -= 8

    # flush remaining bits until we have 22 bytes
    while len(out) < 22:
        out.append(bits & 0xFF)
        bits >>= 8

    return bytes(out[:22])

def make_crsf_frame(sync: int, ftype: int, payload: bytes) -> bytes:
    """
    CRSF frame:
      [0] sync (0xC8 to FC / host -> TX module side is typically 0xC8)
      [1] length = (type + payload + crc) bytes
      [2] type
      [3..] payload
      [end] crc8 over [type+payload]
    """
    length = 1 + len(payload) + 1
    fr = bytearray([sync, length, ftype])
    fr.extend(payload)
    fr.append(crc8_dvb_s2(fr[2:]))  # crc over type+payload
    return bytes(fr)

def disarm(channel=8):
    ch          = [1500] * 16
    ch[2]       = 1000          # throttle minimum
    ch[8:16]    = [1000] * 8   # AUX2-12 off
    ch[9]       = 1000  # 1000 for manual flight mode
    ch[channel] = 1000
    payload = pack_channels_16x11(ch)
    frame = make_crsf_frame(sync=0xC8, ftype=0x16, payload=payload)
    return frame

def arm(channel=8):
    ch          = [1500] * 16
    ch[2]       = 1000          # throttle minimum
    ch[8:16]    = [1000] * 8   # AUX2-12 off
    ch[9]       = 1000  # 1000 for manual flight mode
    ch[channel] = 1800
    payload = pack_channels_16x11(ch)
    frame = make_crsf_frame(sync=0xC8, ftype=0x16, payload=payload)
    return frame

def play(aux= [1100]*4):
    channels          = [1500] * 16
    channels[2]       = 1000          # throttle minimum
    channels[4]       = aux[0]      # AUX1
    channels[5]       = aux[1]      # AUX2
    channels[6]       = aux[2]      # AUX3
    channels[7]       = aux[3]      # AUX4
    channels[8]       = 1800
    channels[9]       = 1000  # 1000 for manual flight mode
    channels[10:16]   = [1000] * 6   # AUX2-12 off
    payload = pack_channels_16x11(channels)
    frame = make_crsf_frame(sync=0xC8, ftype=0x16, payload=payload)
    return frame

# ====== Main ======
def main():
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD,
        bytesize=8,
        parity='N',
        stopbits=1,
        timeout=0,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )

    dt = 1.0 / RATE_HZ
    next_t = time.monotonic()


    print(f"Sending CRSF RC @ {RATE_HZ}Hz on {PORT} @ {BAUD} baud")
    print("CTRL+C to stop")

    try:
        # Establish connection   
        start = time.time()
        while time.time() - start < 6.0:
            frame = disarm()
            ser.write(frame)
            # steady timing
            next_t += dt
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                # if lags, resync to avoid drift
                next_t = time.monotonic()
        buf = deque()

        print("Connection established.")
        while True:

            frame = play()
            ser.write(frame)
            # 16ch: [ch1 roll, ch2 pitch, ch3 throttle, ch4 yaw, ...]
            
            # ch[0]       = 1500          # roll   center
            # ch[1]       = 1500          # pitch  center
            # ch[2]       = 1000          # throttle minimum
            # ch[3]       = 1500          # yaw    center
            # ch[4]       = thr          # AUX1 → motor 1 test value
            # ch[5]      = 1000          # AUX2 off
            # ch[6]      = 1000          # AUX3 off
            # ch[7]      = 1000          # AUX4 off   
            
            # ch[8]       = 1000    # 1000 disarms / 1800 arms
            
            # payload = pack_channels_16x11(ch)

            data = ser.read(64)  # Read up to 64 bytes at a time
            if data:
                buf.extend(data)

            for addr, ftype, payload in frames_from_bytes(buf):
                if ftype in IGNORE_TYPES:
                    continue
                name = FTYPE_NAMES.get(ftype, f"0x{ftype:02X}")
                decoder = _DECODERS.get(ftype)

                if decoder:
                    parsed = decoder(payload)
                    print(f"[{name}] {parsed}")
                else:
                    print(f"[{name}] raw: {payload.hex()}")
            # steady timing
            next_t += dt
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                # if lags, resync to avoid drift
                next_t = time.monotonic()

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("Stopped.")

if __name__ == "__main__":
    main()