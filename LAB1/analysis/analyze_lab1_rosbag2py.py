#!/usr/bin/env python3
import os
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_bag(bag_dir: str, topic: str = "/gps"):
    """
    Reads messages from a ROS2 sqlite3 bag using rosbag2_py and deserializes
    using the installed ROS message type (works for custom msgs).
    Returns a DataFrame with:
      t_s, utm_easting, utm_northing, altitude, hdop, latitude, longitude, zone, letter
    """
    storage_options = StorageOptions(uri=bag_dir, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Map topic -> type string (e.g., 'gps_driver/msg/GpsMsg')
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise RuntimeError(f"Topic {topic} not found in bag {bag_dir}. Found: {list(topic_types.keys())}")

    msg_type_str = topic_types[topic]
    MsgType = get_message(msg_type_str)

    rows = []
    t0_ns = None

    while reader.has_next():
        (tname, data, t_ns) = reader.read_next()
        if tname != topic:
            continue

        msg = deserialize_message(data, MsgType)

        if t0_ns is None:
            t0_ns = t_ns

        rows.append({
            "t_ns": int(t_ns),
            "t_s": (int(t_ns) - int(t0_ns)) * 1e-9,
            "utm_easting": float(msg.utm_easting),
            "utm_northing": float(msg.utm_northing),
            "altitude": float(msg.altitude),
            "hdop": float(msg.hdop),
            "latitude": float(msg.latitude),
            "longitude": float(msg.longitude),
            "zone": int(msg.zone),
            "letter": str(msg.letter),
        })

    df = pd.DataFrame(rows)
    if df.empty:
        raise RuntimeError(f"No messages read for {topic} in {bag_dir}")
    return df.sort_values("t_ns").reset_index(drop=True)


def scatter_en(df, title, outpath):
    e = df["utm_easting"].to_numpy()
    n = df["utm_northing"].to_numpy()
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


def altitude_plot(df, title, outpath):
    plt.figure()
    plt.plot(df["t_s"], df["altitude"])
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.title(title)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(outpath, dpi=200)
    plt.close()


def position_error(df, known_e, known_n):
    de = df["utm_easting"].to_numpy() - known_e
    dn = df["utm_northing"].to_numpy() - known_n
    return np.sqrt(de**2 + dn**2)


def hist_err(err, title, outpath):
    plt.figure()
    plt.hist(err, bins=30)
    plt.xlabel("Position error magnitude (m)")
    plt.ylabel("Count")
    plt.title(title)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(outpath, dpi=200)
    plt.close()


def summarize(label, x):
    print(f"\n[{label}]")
    print(f"  N:      {len(x)}")
    print(f"  Mean:   {np.mean(x):.3f} m")
    print(f"  Std:    {np.std(x):.3f} m")
    print(f"  Median: {np.median(x):.3f} m")
    print(f"  95%:    {np.percentile(x, 95):.3f} m")
    print(f"  Max:    {np.max(x):.3f} m")


def line_fit_perp_error(e, n):
    # fit n = m e + b
    A = np.vstack([e, np.ones_like(e)]).T
    m, b = np.linalg.lstsq(A, n, rcond=None)[0]
    denom = np.sqrt(m*m + 1.0)
    return np.abs(m*e - n + b) / denom


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--occluded", default="occluded_5min")
    ap.add_argument("--open_sky", default="open_sky_5min")
    ap.add_argument("--walking", default="walking_200m")
    ap.add_argument("--topic", default="/gps")

    ap.add_argument("--known_open_e", type=float, required=True)
    ap.add_argument("--known_open_n", type=float, required=True)
    ap.add_argument("--known_occ_e", type=float, required=True)
    ap.add_argument("--known_occ_n", type=float, required=True)
    args = ap.parse_args()

    os.makedirs("plots", exist_ok=True)

    df_occ = read_bag(args.occluded, args.topic)
    df_open = read_bag(args.open_sky, args.topic)
    df_walk = read_bag(args.walking, args.topic)

    # stationary plots
    scatter_en(df_occ, "Occluded (stationary): Northing vs Easting", "plots/occ_scatter_en.png")
    scatter_en(df_open, "Open sky (stationary): Northing vs Easting", "plots/open_scatter_en.png")

    # stationary altitude
    altitude_plot(df_occ, "Occluded (stationary): Altitude vs Time", "plots/occ_altitude.png")
    altitude_plot(df_open, "Open sky (stationary): Altitude vs Time", "plots/open_altitude.png")

    # stationary error histograms
    err_occ = position_error(df_occ, args.known_occ_e, args.known_occ_n)
    err_open = position_error(df_open, args.known_open_e, args.known_open_n)
    hist_err(err_occ, "Occluded (stationary): Position error histogram", "plots/occ_error_hist.png")
    hist_err(err_open, "Open sky (stationary): Position error histogram", "plots/open_error_hist.png")
    summarize("Occluded stationary position error", err_occ)
    summarize("Open sky stationary position error", err_open)

    # walking plots
    scatter_en(df_walk, "Walking: Northing vs Easting", "plots/walk_scatter_en.png")
    altitude_plot(df_walk, "Walking: Altitude vs Time", "plots/walk_altitude.png")

    # walking line-fit error
    e = df_walk["utm_easting"].to_numpy()
    n = df_walk["utm_northing"].to_numpy()
    e = e - e[0]
    n = n - n[0]
    if len(e) >= 2:
        perp = line_fit_perp_error(e, n)
        hist_err(perp, "Walking: Perpendicular error to best-fit line", "plots/walk_linefit_error_hist.png")
        summarize("Walking line-fit perpendicular error", perp)
    else:
        print("\n[Walking] Not enough points for a line fit.")

    # save CSVs
    df_occ.to_csv("plots/occluded_5min.csv", index=False)
    df_open.to_csv("plots/open_sky_5min.csv", index=False)
    df_walk.to_csv("plots/walking_200m.csv", index=False)
    print("\nSaved plots + CSVs in ./plots")


if __name__ == "__main__":
    main()
