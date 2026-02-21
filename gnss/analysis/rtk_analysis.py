#!/usr/bin/env python3
import os
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

# =============================
# CONFIG — CHANGE ONLY IF NEEDED
# =============================
DATA_DIR = os.path.expanduser("~/EECE5554/data")

OPEN_BAG = "openRTK"
OCCLUDED_BAG = "occludedRTK"
WALKING_BAG = "walkingRTK"

TOPIC = "/gps_utm"

# =============================
def load_bag(folder):
    db_files = [f for f in os.listdir(folder) if f.endswith(".db3")]
    if not db_files:
        raise RuntimeError(f"No db3 in {folder}")

    db_path = os.path.join(folder, db_files[0])
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()

    cur.execute("SELECT id, name, type FROM topics")
    topics = cur.fetchall()

    topic_id = None
    msg_type = None
    for tid, name, ttype in topics:
        if name == TOPIC:
            topic_id = tid
            msg_type = ttype
            break

    if topic_id is None:
        raise RuntimeError(f"Topic {TOPIC} not found in {folder}")

    msg_class = get_message(msg_type)

    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=?", (topic_id,))
    rows = cur.fetchall()

    east = []
    north = []
    alt = []
    time = []

    for ts, data in rows:
        msg = deserialize_message(data, msg_class)
        east.append(msg.utm_easting)
        north.append(msg.utm_northing)
        alt.append(msg.altitude)
        time.append(ts * 1e-9)

    return np.array(east), np.array(north), np.array(alt), np.array(time)

# =============================
def centroid_error(e, n):
    ce = np.mean(e)
    cn = np.mean(n)
    dist = np.sqrt((e - ce) ** 2 + (n - cn) ** 2)
    return ce, cn, np.mean(dist)

# =============================
def line_fit_error(e, n):
    p = np.polyfit(e, n, 1)
    fit = np.polyval(p, e)
    err = np.mean(np.abs(n - fit))
    return p, err

# =============================
print("Loading bags...")

e_open, n_open, a_open, t_open = load_bag(os.path.join(DATA_DIR, OPEN_BAG))
e_occ, n_occ, a_occ, t_occ = load_bag(os.path.join(DATA_DIR, OCCLUDED_BAG))
e_walk, n_walk, a_walk, t_walk = load_bag(os.path.join(DATA_DIR, WALKING_BAG))

print("Computing statistics...")

# centroid errors
_, _, err_open = centroid_error(e_open, n_open)
_, _, err_occ = centroid_error(e_occ, n_occ)

# walking line error
p_walk, err_walk = line_fit_error(e_walk, n_walk)

print("\n===== NUMERICAL RESULTS (PUT IN REPORT) =====")
print(f"RTK Open stationary error: {err_open:.3f} m")
print(f"RTK Occluded stationary error: {err_occ:.3f} m")
print(f"RTK Walking line-fit error: {err_walk:.3f} m")

# =============================
# PLOTS
# =============================

os.makedirs("figures", exist_ok=True)

# --- scatter stationary ---
plt.figure(figsize=(6,5))
plt.scatter(e_open - np.mean(e_open), n_open - np.mean(n_open), s=8, label="Open")
plt.scatter(e_occ - np.mean(e_occ), n_occ - np.mean(n_occ), s=8, label="Occluded")
plt.xlabel("Easting error (m)")
plt.ylabel("Northing error (m)")
plt.title("RTK Stationary Scatter")
plt.legend()
plt.grid()
plt.savefig("figures/rtk_stationary_scatter.png", dpi=300)

# --- altitude ---
plt.figure(figsize=(6,5))
plt.plot(t_open - t_open[0], a_open, label="Open")
plt.plot(t_occ - t_occ[0], a_occ, label="Occluded")
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.title("RTK Altitude vs Time")
plt.legend()
plt.grid()
plt.savefig("figures/rtk_altitude.png", dpi=300)

# --- histogram ---
plt.figure(figsize=(10,4))

plt.subplot(1,2,1)
dist_open = np.sqrt((e_open - np.mean(e_open))**2 + (n_open - np.mean(n_open))**2)
plt.hist(dist_open, bins=30)
plt.title("Open Position Error")
plt.xlabel("Error (m)")

plt.subplot(1,2,2)
dist_occ = np.sqrt((e_occ - np.mean(e_occ))**2 + (n_occ - np.mean(n_occ))**2)
plt.hist(dist_occ, bins=30)
plt.title("Occluded Position Error")
plt.xlabel("Error (m)")

plt.tight_layout()
plt.savefig("figures/rtk_histogram.png", dpi=300)

# --- walking scatter ---
plt.figure(figsize=(6,5))
plt.scatter(e_walk, n_walk, s=8, label="Data")
plt.plot(e_walk, np.polyval(p_walk, e_walk), 'r', label="Best fit")
plt.xlabel("Easting (m)")
plt.ylabel("Northing (m)")
plt.title("RTK Walking Path")
plt.legend()
plt.grid()
plt.savefig("figures/rtk_walking.png", dpi=300)

# --- walking altitude ---
plt.figure(figsize=(6,5))
plt.plot(t_walk - t_walk[0], a_walk)
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.title("RTK Walking Altitude")
plt.grid()
plt.savefig("figures/rtk_walking_altitude.png", dpi=300)

print("\n✅ All figures saved to analysis/figures/")
