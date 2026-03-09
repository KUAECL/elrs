import struct
from typing import Deque
from .crc import crc8

ADDRS_START = {0xC8, 0xEA, 0xEE}       # FC, RadioTX, CRSF TX

FT_LINKSTAT = 0x14
FT_BATTERY  = 0x08
FT_GPS      = 0x02
FT_ATTITUDE = 0x1E
FT_VARIO    = 0x07
FT_BARO_ALT = 0x09


def _parse_linkstats(payload: bytes) -> dict:
    if len(payload) != 10:
        return {"error": f"LinkStats invalid length {len(payload)}"}
    keys = ["rssi1_inv", "rssi2_inv", "lq_up", "snr_up",
            "ant", "rf_mode", "txpwr", "rssi_d_inv", "lq_dn", "snr_dn"]
    values = struct.unpack('<BBBBBBBbbb', payload)
    data = dict(zip(keys, values))
    # convert inverted RSSI to real dBm
    data["rssi1_dbm"] = -data["rssi1_inv"]
    data["rssi2_dbm"] = -data["rssi2_inv"]
    data["rssi_d_dbm"] = -abs(data["rssi_d_inv"])
    return data


def _parse_battery(payload: bytes) -> dict:
    """
    Battery-sensor frame (type 0x08)
    u16 voltage   big-endian  (mV * 100)
    u16 current   big-endian  (mA * 100)
    u32 capacity  little-endian:
                  lower 24 bits = capacity [mAh]
                  upper  8 bits = remaining [%]
    """
    if len(payload) != 8:
        return {"error": f"Battery invalid length {len(payload)}"}
    voltage_raw, current_raw = struct.unpack(">HH", payload[:4])
    cap_pack,                = struct.unpack("<I",  payload[4:])
    return {
        "voltage_v":    voltage_raw / 10.0,
        "current_a":    current_raw / 100.0,
        "capacity_mah": cap_pack & 0xFFFFFF,
        "remain_pct":   cap_pack >> 24,
    }


def _parse_gps(payload: bytes) -> dict:
    """
    GPS frame (type 0x02)
    i32 latitude        1e7 degrees
    i32 longitude       1e7 degrees
    u16 groundspeed     cm/s
    u16 heading         centidegrees
    u16 altitude        metres + 1000 offset
    u8  satellites
    """
    if len(payload) != 15:
        return {"error": f"GPS invalid length {len(payload)}"}
    lat, lon, spd, hdg, alt, sats = struct.unpack('>iiHHHB', payload)
    return {
        "lat_deg":        lat / 1e7,
        "lon_deg":        lon / 1e7,
        "groundspeed_ms": spd / 100.0,
        "heading_deg":    hdg / 100.0,
        "altitude_m":     alt - 1000,
        "satellites":     sats,
    }


def _parse_attitude(payload: bytes) -> dict:
    """
    Attitude frame (type 0x1E)
    i16 pitch   radians * 10000
    i16 roll    radians * 10000
    i16 yaw     radians * 10000
    """
    if len(payload) != 6:
        return {"error": f"Attitude invalid length {len(payload)}"}
    pitch_raw, roll_raw, yaw_raw = struct.unpack('>hhh', payload)
    import math
    def r2d(r): return round(math.degrees(r / 10000.0), 2)
    return {
        "pitch_deg": r2d(pitch_raw),
        "roll_deg":  r2d(roll_raw),
        "yaw_deg":   r2d(yaw_raw),
        "pitch_rad": pitch_raw / 10000.0,
        "roll_rad":  roll_raw  / 10000.0,
        "yaw_rad":   yaw_raw   / 10000.0,
    }


def _parse_vario(payload: bytes) -> dict:
    """
    Vario frame (type 0x07)
    i16 vertical speed  cm/s
    """
    if len(payload) != 2:
        return {"error": f"Vario invalid length {len(payload)}"}
    vspd, = struct.unpack('>h', payload)
    return {
        "vspeed_ms": vspd / 100.0,
    }


def _parse_baro_alt(payload: bytes) -> dict:
    """
    Baro altitude frame (type 0x09)
    i16 altitude  decimetres + 10000 offset
    """
    if len(payload) != 2:
        return {"error": f"BaroAlt invalid length {len(payload)}"}
    alt_raw, = struct.unpack('>h', payload)
    return {
        "altitude_m": (alt_raw - 10000) / 10.0,
    }


# Map frame-type → decoder
_DECODERS = {
    FT_LINKSTAT: _parse_linkstats,
    FT_BATTERY:  _parse_battery,
    FT_GPS:      _parse_gps,
    FT_ATTITUDE: _parse_attitude,
    FT_VARIO:    _parse_vario,
    FT_BARO_ALT: _parse_baro_alt,
}

# Human-readable names
FTYPE_NAMES = {
    FT_LINKSTAT: "linkstats",
    FT_BATTERY:  "battery",
    FT_GPS:      "gps",
    FT_ATTITUDE: "attitude",
    FT_VARIO:    "vario",
    FT_BARO_ALT: "baro_alt",
}


def frames_from_bytes(buf: Deque[int]):
    """
    In-place parser – consumes bytes from *buf* and yields complete frames.
    Yields: (addr, ftype, payload_bytes)
    """
    while True:
        if len(buf) < 3:
            return

        if buf[0] not in ADDRS_START:
            buf.popleft()
            continue

        size = buf[1]
        frame_total = size + 2
        if len(buf) < frame_total:
            return

        crc_in   = buf[frame_total - 1]
        calc_crc = crc8(bytes(list(buf)[2:frame_total - 1]))
        if crc_in != calc_crc:
            buf.popleft()
            continue

        addr    = buf.popleft()
        _       = buf.popleft()          # size, discard
        ftype   = buf.popleft()
        payload = bytes(buf.popleft() for _ in range(frame_total - 3 - 1))
        buf.popleft()                    # CRC byte
        yield addr, ftype, payload