#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

BAG_STATIC = Path("bags/gps_static")
BAG_WALK = Path("bags/gps_walk")
TOPIC = "/gps_utm"
MSG_TYPE = "gps_interfaces/msg/GpsUtm"

def read_bag(bag_dir: Path):
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("cdr", "cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    msg_cls = get_message(MSG_TYPE)
    easting, northing = [], []

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != TOPIC:
            continue
        msg = deserialize_message(data, msg_cls)
        easting.append(msg.easting)
        northing.append(msg.northing)

    return np.array(easting), np.array(northing)

E_static, N_static = read_bag(BAG_STATIC)
E_walk, N_walk = read_bag(BAG_WALK)

print("\n--- STATIONARY GPS STATISTICS ---")
print(f"Mean Easting  : {np.mean(E_static):.3f} m")
print(f"Std Easting   : {np.std(E_static):.3f} m")
print(f"Mean Northing : {np.mean(N_static):.3f} m")
print(f"Std Northing  : {np.std(N_static):.3f} m")

plt.figure()
plt.scatter(E_static, N_static, s=10)
plt.xlabel("Easting (m)")
plt.ylabel("Northing (m)")
plt.title("Stationary GPS Scatter")
plt.axis("equal")
plt.grid(True)

plt.figure()
plt.plot(E_walk, N_walk, linewidth=2)
plt.xlabel("Easting (m)")
plt.ylabel("Northing (m)")
plt.title("Walking GPS Path")
plt.axis("equal")
plt.grid(True)

plt.figure()
plt.plot(E_static, label="Easting")
plt.plot(N_static, label="Northing")
plt.xlabel("Sample Index")
plt.ylabel("Meters")
plt.title("Stationary GPS Samples")
plt.legend()
plt.grid(True)

plt.show()
