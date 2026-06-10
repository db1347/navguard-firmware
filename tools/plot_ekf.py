#!/usr/bin/env python3
"""Plot EKF output vs raw GPS from navguard_replay CSV.

    ./build-desktop/navguard_replay test_data/tel_aviv_drive.nmea > out.csv
    python3 tools/plot_ekf.py out.csv

Requires matplotlib. Falls back to a text summary if it is not installed.
"""
import csv
import sys


def load(path):
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            rows.append(row)
    return rows


def main():
    if len(sys.argv) < 2:
        print("usage: plot_ekf.py <out.csv>", file=sys.stderr)
        sys.exit(2)
    rows = load(sys.argv[1])
    if not rows:
        print("no data", file=sys.stderr)
        sys.exit(1)

    states = {}
    for r in rows:
        states[r["state"]] = states.get(r["state"], 0) + 1

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print(f"{len(rows)} fixes")
        print("state histogram:", states)
        print("(install matplotlib to see the track plot)")
        return

    gps_lat = [float(r["gps_lat"]) for r in rows]
    gps_lon = [float(r["gps_lon"]) for r in rows]
    ekf_lat = [float(r["ekf_lat"]) for r in rows]
    ekf_lon = [float(r["ekf_lon"]) for r in rows]

    plt.figure(figsize=(9, 7))
    plt.plot(gps_lon, gps_lat, "r.", label="raw GPS", alpha=0.5)
    plt.plot(ekf_lon, ekf_lat, "b-", label="EKF fused")
    plt.xlabel("longitude")
    plt.ylabel("latitude")
    plt.title(f"NavGuard EKF track ({len(rows)} fixes)  states={states}")
    plt.legend()
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.show()


if __name__ == "__main__":
    main()
