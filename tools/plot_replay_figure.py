#!/usr/bin/env python3
"""Render the README figure from a navguard_replay CSV.

    ./build-desktop/navguard_replay test_data/tel_aviv_spoof.nmea > out.csv
    python3 tools/plot_replay_figure.py out.csv docs/replay_spoof.png

Top panel: distance of raw GPS and of the EKF output from the starting point.
Bottom panel: detector state and the GPS weight the EKF is using.
"""
import csv
import math
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

EARTH_RADIUS_KM = 6371.0
STATE_LEVEL = {"GPS_LOCK": 3, "LOCK": 3, "GPS_SUSPECT": 2, "SUSPECT": 2,
               "GPS_SPOOFED": 1, "SPOOFED": 1, "DR_ONLY": 0}


def haversine_km(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dlat, dlon = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_KM * math.asin(math.sqrt(a))


def main():
    if len(sys.argv) != 3:
        print("usage: plot_replay_figure.py <out.csv> <figure.png>", file=sys.stderr)
        sys.exit(2)
    with open(sys.argv[1]) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit("no data")

    t0 = int(rows[0]["ts_ms"])
    lat0, lon0 = float(rows[0]["gps_lat"]), float(rows[0]["gps_lon"])
    t = [(int(r["ts_ms"]) - t0) / 1000 for r in rows]
    gps_km = [haversine_km(lat0, lon0, float(r["gps_lat"]), float(r["gps_lon"])) for r in rows]
    ekf_km = [haversine_km(lat0, lon0, float(r["ekf_lat"]), float(r["ekf_lon"])) for r in rows]
    level = [STATE_LEVEL.get(r["state"], 0) for r in rows]
    weight = [float(r["gps_weight"]) for r in rows]

    fig, (top, bottom) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True,
                                      gridspec_kw={"height_ratios": [3, 2]})
    top.plot(t, gps_km, color="#d62728", lw=2, label="raw GPS (spoofed after t=15 s)")
    top.plot(t, ekf_km, color="#1f77b4", lw=2, label="EKF output")
    top.set_ylabel("distance from start [km]")
    top.set_title("NavGuard replay: GPS teleported from Tel Aviv to Beirut at t = 15 s")
    top.legend(loc="upper left")
    top.grid(alpha=0.3)

    bottom.step(t, level, where="post", color="black", lw=2, label="detector state")
    bottom.set_yticks([0, 1, 2, 3], ["DR_ONLY", "SPOOFED", "SUSPECT", "LOCK"])
    bottom.set_ylim(-0.3, 3.5)
    twin = bottom.twinx()
    twin.step(t, weight, where="post", color="#2ca02c", lw=1.5, ls="--", label="GPS weight")
    twin.set_ylim(-0.08, 1.0)
    twin.set_ylabel("GPS weight in EKF", color="#2ca02c")
    bottom.set_xlabel("time [s]")
    bottom.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(sys.argv[2], dpi=150)
    print(f"wrote {sys.argv[2]}")


if __name__ == "__main__":
    main()
