#!/usr/bin/env python3
import os
import math
import argparse
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rosidl_runtime_py.utilities import get_message


def read_bag_topic(bag_dir: str, topic_name: str):
    """
    Reads a rosbag2 (sqlite3) directory and returns:
    t_sec (relative), easting, northing, altitude, hdop, fix_quality
    Works as long as the message has attributes:
      utm_easting, utm_northing, altitude
    and optionally:
      hdop, fix_quality, header.stamp
    """
    if not os.path.isdir(bag_dir):
        raise FileNotFoundError(f"Bag directory not found: {bag_dir}")

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_dir, storage_id="sqlite3")
    converter_options = ConverterOptions(input_serialization_format="cdr",
                                         output_serialization_format="cdr")
    reader.open(storage_options, converter_options)

    # Map topics -> types
    topics = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics}

    if topic_name not in type_map:
        available = "\n".join(sorted(type_map.keys()))
        raise RuntimeError(f"Topic '{topic_name}' not in bag.\nAvailable topics:\n{available}")

    type_str = type_map[topic_name]
    try:
        msg_type = get_message(type_str)
    except ModuleNotFoundError as e:
        # Common Lab1 situation: bag says gps_driver/msg/GpsMsg, but Python msgs are in gps_interfaces
        if type_str.endswith("/GpsMsg"):
            msg_type = get_message("gps_interfaces/msg/GpsMsg")
        else:
            raise e

    # Filter to only our topic

    reader.set_filter(StorageFilter(topics=[topic_name]))


    t_list = []
    e_list, n_list, alt_list = [], [], []
    hdop_list, fixq_list = [], []

    t0 = None

    while reader.has_next():
        topic, data, t_ns = reader.read_next()  # t_ns is bag time in nanoseconds
        msg = deserialize_message(data, msg_type)

        # Use header stamp if present, else bag time
        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            t = t_ns * 1e-9

        if t0 is None:
            t0 = t
        t_list.append(t - t0)

        e_list.append(float(getattr(msg, "utm_easting")))
        n_list.append(float(getattr(msg, "utm_northing")))
        alt_list.append(float(getattr(msg, "altitude")))

        hdop_list.append(float(getattr(msg, "hdop", np.nan)))
        fixq_list.append(int(getattr(msg, "fix_quality", -1)))

    return (np.array(t_list),
            np.array(e_list),
            np.array(n_list),
            np.array(alt_list),
            np.array(hdop_list),
            np.array(fixq_list))


def centroid_error(e, n):
    ce = np.mean(e)
    cn = np.mean(n)
    d = np.sqrt((e - ce)**2 + (n - cn)**2)
    return ce, cn, d


def line_fit_error(e, n):
    """
    Fit a line n = m*e + b and compute perpendicular distances.
    Returns m, b, distances, RMS distance.
    """
    m, b = np.polyfit(e, n, 1)
    # Distance from point (e_i,n_i) to line m*e - n + b = 0
    # line: m*e - n + b = 0 -> ax + by + c = 0 with a=m, b=-1, c=b
    a = m
    bb = -1.0
    c = b
    dist = np.abs(a*e + bb*n + c) / np.sqrt(a*a + bb*bb)
    rms = np.sqrt(np.mean(dist**2))
    return m, b, dist, rms


def savefig(path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    plt.tight_layout()
    plt.savefig(path, dpi=200)
    plt.close()


def stationary_plots(open_bag, occ_bag, topic, outdir):
    # Read
    tO, eO, nO, aO, hdO, fqO = read_bag_topic(open_bag, topic)
    tC, eC, nC, aC, hdC, fqC = read_bag_topic(occ_bag, topic)

    # Centroids + distances
    ceO, cnO, dO = centroid_error(eO, nO)[0:3]
    ceC, cnC, dC = centroid_error(eC, nC)[0:3]

    # Demean for scatter
    eO0, nO0 = eO - ceO, nO - cnO
    eC0, nC0 = eC - ceC, nC - cnC

    # Offsets (mean easting/northing)
    offO = (ceO, cnO)
    offC = (ceC, cnC)

    # --- Plot 1: Stationary scatter (demeaned) ---
    plt.figure()
    plt.scatter(eO0, nO0, s=10, label="Open RTK (demeaned)")
    plt.scatter(eC0, nC0, s=10, label="Occluded RTK (demeaned)")
    plt.xlabel("Easting offset from centroid (m)")
    plt.ylabel("Northing offset from centroid (m)")
    plt.title("RTK Stationary: Northing vs Easting (centroid removed)")
    plt.legend()
    # Put centroid offsets as text
    txt = (f"Open centroid:  E={offO[0]:.3f} m, N={offO[1]:.3f} m\n"
           f"Occ  centroid:  E={offC[0]:.3f} m, N={offC[1]:.3f} m\n"
           f"Open mean HDOP={np.nanmean(hdO):.2f}, fixQ≈{int(np.median(fqO))}\n"
           f"Occ  mean HDOP={np.nanmean(hdC):.2f}, fixQ≈{int(np.median(fqC))}")
    plt.gcf().text(0.02, 0.02, txt, fontsize=9)
    savefig(os.path.join(outdir, "rtk_stationary_scatter_demeaned.png"))

    # --- Plot 2: Stationary altitude vs time ---
    plt.figure()
    plt.plot(tO, aO, label="Open RTK")
    plt.plot(tC, aC, label="Occluded RTK")
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.title("RTK Stationary: Altitude vs Time")
    plt.legend()
    savefig(os.path.join(outdir, "rtk_stationary_altitude_vs_time.png"))

    # --- Plot 3: Histograms of distance from centroid ---
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.hist(dO, bins=30)
    plt.xlabel("Distance from centroid (m)")
    plt.ylabel("Count")
    plt.title("Open RTK: Histogram of distance from centroid")

    plt.subplot(2, 1, 2)
    plt.hist(dC, bins=30)
    plt.xlabel("Distance from centroid (m)")
    plt.ylabel("Count")
    plt.title("Occluded RTK: Histogram of distance from centroid")
    savefig(os.path.join(outdir, "rtk_stationary_hist_distance.png"))

    # Return numeric summary
    open_rms = float(np.sqrt(np.mean(dO**2)))
    occ_rms = float(np.sqrt(np.mean(dC**2)))
    return {
        "open_centroid_e": float(offO[0]),
        "open_centroid_n": float(offO[1]),
        "open_mean_hdop": float(np.nanmean(hdO)),
        "open_fixq_median": int(np.median(fqO)),
        "open_rms_error_m": open_rms,

        "occ_centroid_e": float(offC[0]),
        "occ_centroid_n": float(offC[1]),
        "occ_mean_hdop": float(np.nanmean(hdC)),
        "occ_fixq_median": int(np.median(fqC)),
        "occ_rms_error_m": occ_rms,
    }


def moving_plots(walk_bag, topic, outdir):
    tW, eW, nW, aW, hdW, fqW = read_bag_topic(walk_bag, topic)

    # Line fit in NE plane: northing vs easting
    m, b, dist, rms = line_fit_error(eW, nW)

    # Scatter with fitted line
    plt.figure()
    plt.plot(eW, nW, marker="o", linestyle="-", markersize=3, linewidth=1, label="Walking RTK path")
    e_line = np.array([np.min(eW), np.max(eW)])
    n_line = m * e_line + b
    plt.plot(e_line, n_line, linestyle="--", label="Best-fit line")
    plt.xlabel("UTM Easting (m)")
    plt.ylabel("UTM Northing (m)")
    plt.title("RTK Walking: Northing vs Easting + Best-fit line")
    plt.legend()
    txt = (f"Best-fit: N = {m:.6f}*E + {b:.3f}\n"
           f"RMS perpendicular error = {rms:.3f} m\n"
           f"Mean HDOP={np.nanmean(hdW):.2f}, fixQ≈{int(np.median(fqW))}")
    plt.gcf().text(0.02, 0.02, txt, fontsize=9)
    savefig(os.path.join(outdir, "rtk_walking_NE_with_fit.png"))

    # Altitude vs time
    plt.figure()
    plt.plot(tW, aW)
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.title("RTK Walking: Altitude vs Time")
    savefig(os.path.join(outdir, "rtk_walking_altitude_vs_time.png"))

    return {
        "walk_line_m": float(m),
        "walk_line_b": float(b),
        "walk_rms_error_m": float(rms),
        "walk_mean_hdop": float(np.nanmean(hdW)),
        "walk_fixq_median": int(np.median(fqW)),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--open_rtk", required=True, help="Path to open RTK bag directory")
    parser.add_argument("--occluded_rtk", required=True, help="Path to occluded RTK bag directory")
    parser.add_argument("--walking_rtk", required=True, help="Path to walking RTK bag directory")
    parser.add_argument("--topic", default="/gps_utm", help="Topic name (default /gps_utm)")
    parser.add_argument("--outdir", default="figures", help="Output directory for plots")
    args = parser.parse_args()

    rclpy.init(args=None)

    outdir = os.path.abspath(args.outdir)
    os.makedirs(outdir, exist_ok=True)

    stat = stationary_plots(args.open_rtk, args.occluded_rtk, args.topic, outdir)
    mov = moving_plots(args.walking_rtk, args.topic, outdir)

    print("\n===== RTK RESULTS (numbers you will put in report) =====")
    print(f"Open RTK:    RMS error from centroid = {stat['open_rms_error_m']:.3f} m")
    print(f"             mean HDOP = {stat['open_mean_hdop']:.2f}, median fixQ = {stat['open_fixq_median']}")
    print(f"Occluded RTK:RMS error from centroid = {stat['occ_rms_error_m']:.3f} m")
    print(f"             mean HDOP = {stat['occ_mean_hdop']:.2f}, median fixQ = {stat['occ_fixq_median']}")
    print(f"Walking RTK: RMS error from best-fit line = {mov['walk_rms_error_m']:.3f} m")
    print(f"             mean HDOP = {mov['walk_mean_hdop']:.2f}, median fixQ = {mov['walk_fixq_median']}")
    print(f"\nPlots saved to: {outdir}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
