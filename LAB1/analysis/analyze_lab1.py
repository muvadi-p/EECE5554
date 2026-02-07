#!/usr/bin/env python3
import os
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr


def load_bag_topic(bag_dir: str, topic: str = "/gps") -> pd.DataFrame:
    """
    Reads /gps messages from a ROS2 bag directory and returns a DataFrame
    with time (s), utm_easting, utm_northing, altitude, hdop, lat, lon.
    """
    rows = []
    with Reader(bag_dir) as reader:
        # Find connection for the topic
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise RuntimeError(f"Topic {topic} not found in {bag_dir}")

        for conn, t_ns, raw in reader.messages(connections=conns):
            msg = deserialize_cdr(raw, conn.msgtype)

            # ROS2 bag timestamp is t_ns (record time). Header stamp is inside msg.header.stamp.
            # Lab wants header based on GPS time; but for plotting, record time is OK for relative time.
            # We'll use bag time for x-axis and relative scaling.

            rows.append({
                "t_ns": int(t_ns),
                "utm_easting": float(msg.utm_easting),
                "utm_northing": float(msg.utm_northing),
                "altitude": float(msg.altitude),
                "hdop": float(msg.hdop),
                "latitude": float(msg.latitude),
                "longitude": float(msg.longitude),
                "zone": int(msg.zone),
                "letter": str(msg.letter),
            })

    df = pd.DataFrame(rows).sort_values("t_ns").reset_index(drop=True)
    if df.empty:
        raise RuntimeError(f"No messages read from {bag_dir} on {topic}")

    # Relative time seconds from first message
    df["t_s"] = (df["t_ns"] - df["t_ns"].iloc[0]) * 1e-9
    return df


def position_error_m(df: pd.DataFrame, known_e: float, known_n: float) -> np.ndarray:
    de = df["utm_easting"].to_numpy() - known_e
    dn = df["utm_northing"].to_numpy() - known_n
    return np.sqrt(de**2 + dn**2)


def line_fit_perp_error_m(e: np.ndarray, n: np.ndarray) -> np.ndarray:
    """
    Fit line n = m*e + b (least squares) and return perpendicular distances to the line.
    """
    A = np.vstack([e, np.ones_like(e)]).T
    m, b = np.linalg.lstsq(A, n, rcond=None)[0]
    # distance from point (e_i, n_i) to line m*e - n + b = 0
    # line: m*e - n + b = 0 => ax + by + c = 0 with a=m, b=-1, c=b
    denom = np.sqrt(m**2 + 1.0)
    d = np.abs(m*e - n + b) / denom
    return d


def scatter_en(df: pd.DataFrame, title: str, outpath: str, subtract_first: bool = True):
    e = df["utm_easting"].to_numpy()
    n = df["utm_northing"].to_numpy()
    if subtract_first:
        e = e - e[0]
        n = n - n[0]

    plt.figure()
    plt.scatter(e, n, s=12)
    plt.xlabel("Easting (m) [first sample subtracted]")
    plt.ylabel("Northing (m) [first sample subtracted]")
    plt.title(title)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(outpath, dpi=200)
    plt.close()


def plot_altitude(df: pd.DataFrame, title: str, outpath: str):
    plt.figure()
    plt.plot(df["t_s"], df["altitude"])
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.title(title)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(outpath, dpi=200)
    plt.close()


def hist_error(err: np.ndarray, title: str, outpath: str):
    plt.figure()
    plt.hist(err, bins=30)
    plt.xlabel("Position error magnitude (m)")
    plt.ylabel("Count")
    plt.title(title)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(outpath, dpi=200)
    plt.close()


def summarize_errors(label: str, err: np.ndarray):
    print(f"\n[{label}] Position error summary (meters):")
    print(f"  N samples: {len(err)}")
    print(f"  Mean:      {np.mean(err):.3f}")
    print(f"  Std:       {np.std(err):.3f}")
    print(f"  Median:    {np.median(err):.3f}")
    print(f"  95th pct:  {np.percentile(err, 95):.3f}")
    print(f"  Max:       {np.max(err):.3f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--occluded", default="occluded_5min", help="Bag directory for occluded stationary data")
    ap.add_argument("--open_sky", default="open_sky_5min", help="Bag directory for open sky stationary data")
    ap.add_argument("--walking", default="walking_200m", help="Bag directory for walking data")
    ap.add_argument("--topic", default="/gps", help="Topic name")
    ap.add_argument("--known_open_e", type=float, required=True, help="Known UTM easting for open sky location (m)")
    ap.add_argument("--known_open_n", type=float, required=True, help="Known UTM northing for open sky location (m)")
    ap.add_argument("--known_occ_e", type=float, required=True, help="Known UTM easting for occluded location (m)")
    ap.add_argument("--known_occ_n", type=float, required=True, help="Known UTM northing for occluded location (m)")
    args = ap.parse_args()

    os.makedirs("plots", exist_ok=True)

    # Load bags
    df_occ = load_bag_topic(args.occluded, args.topic)
    df_open = load_bag_topic(args.open_sky, args.topic)
    df_walk = load_bag_topic(args.walking, args.topic)

    # Scatterplots stationary
    scatter_en(df_occ, "Occluded (stationary): Northing vs Easting", "plots/occ_scatter_en.png")
    scatter_en(df_open, "Open sky (stationary): Northing vs Easting", "plots/open_scatter_en.png")

    # Errors to known points (single error magnitude per sample)
    err_occ = position_error_m(df_occ, args.known_occ_e, args.known_occ_n)
    err_open = position_error_m(df_open, args.known_open_e, args.known_open_n)

    hist_error(err_occ, "Occluded (stationary): Position error histogram", "plots/occ_error_hist.png")
    hist_error(err_open, "Open sky (stationary): Position error histogram", "plots/open_error_hist.png")

    summarize_errors("Occluded", err_occ)
    summarize_errors("Open sky", err_open)

    # Altitude vs time
    plot_altitude(df_occ, "Occluded (stationary): Altitude vs Time", "plots/occ_altitude.png")
    plot_altitude(df_open, "Open sky (stationary): Altitude vs Time", "plots/open_altitude.png")
    plot_altitude(df_walk, "Walking: Altitude vs Time", "plots/walk_altitude.png")

    # Walking: scatter and line-fit error
    scatter_en(df_walk, "Walking: Northing vs Easting", "plots/walk_scatter_en.png")

    e = df_walk["utm_easting"].to_numpy()
    n = df_walk["utm_northing"].to_numpy()
    e0, n0 = e[0], n[0]
    e_s = e - e0
    n_s = n - n0

    if len(e_s) >= 2:
        perp_err = line_fit_perp_error_m(e_s, n_s)
        print(f"\n[Walking] Line-fit perpendicular error (meters):")
        print(f"  N samples: {len(perp_err)}")
        print(f"  Mean:      {np.mean(perp_err):.3f}")
        print(f"  Std:       {np.std(perp_err):.3f}")
        print(f"  95th pct:  {np.percentile(perp_err, 95):.3f}")
        print(f"  Max:       {np.max(perp_err):.3f}")

        hist_error(perp_err, "Walking: Perpendicular error to best-fit line", "plots/walk_linefit_error_hist.png")
    else:
        print("\n[Walking] Not enough points for line fit.")

    # Save a CSV summary for convenience
    df_occ.to_csv("plots/occluded_5min.csv", index=False)
    df_open.to_csv("plots/open_sky_5min.csv", index=False)
    df_walk.to_csv("plots/walking_200m.csv", index=False)
    print("\nSaved plots + CSVs in ./plots")


if __name__ == "__main__":
    main()
