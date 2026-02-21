#!/usr/bin/env python3
import os
import numpy as np
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


FIG_DIR = os.path.join(os.path.dirname(__file__), "figures")
os.makedirs(FIG_DIR, exist_ok=True)


def read_bag_xyzt(bag_path, topic_name="/gps"):
    """
    Reads /gps (gps_driver/msg/GpsMsg) from a rosbag2 sqlite3 bag folder.
    Returns arrays: t_sec, easting, northing, altitude
    """
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Resolve type for the topic
    topics = reader.get_all_topics_and_types()
    topic_type = None
    for t in topics:
        if t.name == topic_name:
            topic_type = t.type
            break
    if topic_type is None:
        raise RuntimeError(f"Topic {topic_name} not found in bag {bag_path}. Found: {[t.name for t in topics]}")

    msg_type = get_message(topic_type)

    t_list, e_list, n_list, a_list = [], [], [], []
    while reader.has_next():
        (topic, data, _) = reader.read_next()
        if topic != topic_name:
            continue
        msg = deserialize_message(data, msg_type)

        # Time from header stamp if available, else fall back to None
        # Lab1 messages usually have header.stamp
        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            t_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        else:
            t_sec = np.nan

        # Common fields in your Lab1 message (based on your prior outputs)
        # utm_easting, utm_northing, altitude should exist
        e = float(getattr(msg, "utm_easting"))
        n = float(getattr(msg, "utm_northing"))
        a = float(getattr(msg, "altitude"))

        t_list.append(t_sec)
        e_list.append(e)
        n_list.append(n)
        a_list.append(a)

    t = np.array(t_list)
    e = np.array(e_list)
    n = np.array(n_list)
    a = np.array(a_list)

    # Normalize time to start at 0 if we have real times
    if np.all(np.isfinite(t)) and len(t) > 0:
        t = t - t[0]

    return t, e, n, a


def rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float("nan")


def stationary_metrics(e, n):
    e0 = float(np.mean(e))
    n0 = float(np.mean(n))
    de = e - e0
    dn = n - n0
    d = np.sqrt(de**2 + dn**2)
    return (e0, n0, de, dn, d, rms(d), float(np.mean(d)))


def walking_linefit_error(e, n):
    """
    Fit n = m*e + b, compute perpendicular distance to line, return RMS distance in meters.
    """
    if len(e) < 2:
        return float("nan"), (np.nan, np.nan), np.array([])

    m, b = np.polyfit(e, n, 1)

    # Distance from point (e_i, n_i) to line: m*e - n + b = 0
    # dist = |m*e_i - n_i + b| / sqrt(m^2 + 1)
    dist = np.abs(m * e - n + b) / np.sqrt(m**2 + 1.0)
    return rms(dist), (m, b), dist


def savefig(name):
    out = os.path.join(FIG_DIR, name)
    plt.savefig(out, dpi=200, bbox_inches="tight")
    plt.close()


def main():
    print("Loading standalone GNSS bags...")

    bag_open = os.path.expanduser("~/EECE5554/data/openGNSS")
    bag_occ  = os.path.expanduser("~/EECE5554/data/occludedGNSS")
    bag_walk = os.path.expanduser("~/EECE5554/data/walkingGNSS")

    # Read bags
    t_open, e_open, n_open, a_open = read_bag_xyzt(bag_open, "/gps")
    t_occ,  e_occ,  n_occ,  a_occ  = read_bag_xyzt(bag_occ,  "/gps")
    t_w,    e_w,    n_w,    a_w    = read_bag_xyzt(bag_walk, "/gps")

    print("Computing statistics...")

    # Stationary metrics
    e0_open, n0_open, de_open, dn_open, d_open, rms_open, mean_open = stationary_metrics(e_open, n_open)
    e0_occ,  n0_occ,  de_occ,  dn_occ,  d_occ,  rms_occ,  mean_occ  = stationary_metrics(e_occ,  n_occ)

    # Walking line fit
    walk_rms, (m, b), dist = walking_linefit_error(e_w, n_w)

    print("\n===== NUMERICAL RESULTS (PUT IN REPORT) =====")
    print(f"Standalone Open stationary error (RMS): {rms_open:.3f} m")
    print(f"Standalone Occluded stationary error (RMS): {rms_occ:.3f} m")
    print(f"Standalone Walking line-fit error (RMS): {walk_rms:.3f} m")

    # ---- PLOTS ----

    # 1) Stationary scatter (centroid subtracted) + annotate offsets
    plt.figure()
    plt.scatter(de_open, dn_open, s=10, label="Open (centroid-subtracted)")
    plt.scatter(de_occ, dn_occ, s=10, label="Occluded (centroid-subtracted)")
    plt.xlabel("Easting offset from centroid (m)")
    plt.ylabel("Northing offset from centroid (m)")
    plt.title("Standalone GNSS: Stationary Scatter (Centroid-Subtracted)")
    plt.legend()
    txt = (
        f"Open centroid:  E={e0_open:.3f}, N={n0_open:.3f}\n"
        f"Occl centroid:  E={e0_occ:.3f}, N={n0_occ:.3f}\n"
        f"Centroid offset (Occl-Open): ΔE={e0_occ-e0_open:.3f} m, ΔN={n0_occ-n0_open:.3f} m"
    )
    plt.gcf().text(0.02, 0.02, txt, fontsize=9)
    savefig("gnss_stationary_scatter.png")

    # 2) Stationary altitude vs time (open+occluded)
    plt.figure()
    plt.plot(t_open, a_open, label="Open")
    plt.plot(t_occ, a_occ, label="Occluded")
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.title("Standalone GNSS: Stationary Altitude vs Time")
    plt.legend()
    savefig("gnss_altitude.png")

    # 3) Stationary histogram distance-from-centroid (open+occluded in subplots)
    plt.figure(figsize=(10, 4))
    plt.subplot(1, 2, 1)
    plt.hist(d_open, bins=30)
    plt.xlabel("Distance from centroid (m)")
    plt.ylabel("Count")
    plt.title(f"Open (RMS={rms_open:.3f} m)")

    plt.subplot(1, 2, 2)
    plt.hist(d_occ, bins=30)
    plt.xlabel("Distance from centroid (m)")
    plt.ylabel("Count")
    plt.title(f"Occluded (RMS={rms_occ:.3f} m)")

    plt.suptitle("Standalone GNSS: Stationary Position Error Histograms")
    savefig("gnss_histogram.png")

    # 4) Walking N vs E scatter + best fit line
    plt.figure()
    plt.scatter(e_w, n_w, s=12, label="Walking data")
    if np.isfinite(m) and np.isfinite(b):
        e_line = np.linspace(np.min(e_w), np.max(e_w), 200)
        n_line = m * e_line + b
        plt.plot(e_line, n_line, label=f"Best fit: N = {m:.3f}E + {b:.3f}")
    plt.xlabel("UTM Easting (m)")
    plt.ylabel("UTM Northing (m)")
    plt.title(f"Standalone GNSS: Walking Scatter + Best Fit (RMS dist={walk_rms:.3f} m)")
    plt.legend()
    savefig("gnss_walking.png")

    # 5) Walking altitude vs time
    plt.figure()
    plt.plot(t_w, a_w)
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.title("Standalone GNSS: Walking Altitude vs Time")
    savefig("gnss_walking_altitude.png")

    print(f"\n✅ Standalone figures saved to: {FIG_DIR}")


if __name__ == "__main__":
    main()
