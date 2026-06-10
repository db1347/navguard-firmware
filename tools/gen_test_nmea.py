#!/usr/bin/env python3
"""Generate a synthetic Tel Aviv drive as NMEA (GGA+RMC at 10Hz).

This is a stand-in for a real recording until one is captured with GPS Logger
(see dev plan Section 5.4). Produces a ~60s eastbound drive plus an optional
spoofing event (a sudden jump to Beirut) for spoof-detector verification.

    python3 tools/gen_test_nmea.py > test_data/tel_aviv_drive.nmea
    python3 tools/gen_test_nmea.py --spoof > test_data/tel_aviv_spoof.nmea
"""
import argparse
import math
import sys

METERS_PER_DEG_LAT = 111111.0


def nmea_checksum(body: str) -> str:
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"{cs:02X}"


def fmt_lat(lat: float) -> tuple[str, str]:
    hemi = "N" if lat >= 0 else "S"
    a = abs(lat)
    d = int(a)
    m = (a - d) * 60.0
    return f"{d:02d}{m:09.6f}", hemi


def fmt_lon(lon: float) -> tuple[str, str]:
    hemi = "E" if lon >= 0 else "W"
    a = abs(lon)
    d = int(a)
    m = (a - d) * 60.0
    return f"{d:03d}{m:09.6f}", hemi


def emit(lat, lon, speed_mps, heading, t_s):
    hh = int(t_s // 3600) % 24
    mm = int((t_s // 60) % 60)
    ss = t_s % 60
    tstr = f"{hh:02d}{mm:02d}{ss:05.2f}"
    lat_s, ns = fmt_lat(lat)
    lon_s, ew = fmt_lon(lon)
    knots = speed_mps / 0.514444

    gga_body = (f"GNGGA,{tstr},{lat_s},{ns},{lon_s},{ew},1,12,0.8,"
                f"35.0,M,17.0,M,,")
    rmc_body = (f"GNRMC,{tstr},A,{lat_s},{ns},{lon_s},{ew},"
                f"{knots:.1f},{heading:.1f},100624,,,A")
    print(f"${rmc_body}*{nmea_checksum(rmc_body)}")
    print(f"${gga_body}*{nmea_checksum(gga_body)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spoof", action="store_true",
                    help="inject a Beirut jump at t=15s")
    ap.add_argument("--seconds", type=float, default=60.0)
    args = ap.parse_args()

    lat, lon = 32.0853, 34.7818  # central Tel Aviv
    speed = 13.4                 # ~48 km/h
    heading = 90.0               # due east
    dt = 0.1
    t = 43200.0                  # 12:00:00

    steps = int(args.seconds / dt)
    for i in range(steps):
        if args.spoof and i == int(15.0 / dt):
            # Spoofer teleports the receiver to Beirut.
            print("# --- spoofing event: jump to Beirut ---", file=sys.stderr)
            lat, lon = 33.8938, 35.5018
        emit(lat, lon, speed, heading, t)
        # advance ground truth east
        lon += (speed * dt) / (METERS_PER_DEG_LAT *
                               math.cos(math.radians(lat)))
        t += dt


if __name__ == "__main__":
    main()
