import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

bag = Path("../Data/imu_bag_1_0.mcap")

if not bag.exists():
    raise FileNotFoundError(f"Bag not found: {bag}")

gyro_x = []
gyro_y = []
gyro_z = []
time = []

typestore = get_typestore(Stores.ROS2_HUMBLE)

with AnyReader([bag], default_typestore=typestore) as reader:
    for connection, timestamp, rawdata in reader.messages():
        msg = reader.deserialize(rawdata, connection.msgtype)

        # custom_interfaces/msg/IMUmsg stores sensor_msgs/Imu inside msg.imu
        if hasattr(msg, "imu"):
            t = timestamp * 1e-9
            time.append(t)

            gyro_x.append(msg.imu.angular_velocity.x)
            gyro_y.append(msg.imu.angular_velocity.y)
            gyro_z.append(msg.imu.angular_velocity.z)

time = np.array(time)
time = time - time[0]

gyro_x = np.array(gyro_x)
gyro_y = np.array(gyro_y)
gyro_z = np.array(gyro_z)

if len(time) == 0:
    raise RuntimeError("No IMU data found in motion bag.")

def plot_clip(start, end, name):
    mask = (time >= start) & (time <= end)

    plt.figure(figsize=(10, 6))
    plt.plot(time[mask], gyro_x[mask], label="Gyro X")
    plt.plot(time[mask], gyro_y[mask], label="Gyro Y")
    plt.plot(time[mask], gyro_z[mask], label="Gyro Z")

    plt.xlabel("Time (s)")
    plt.ylabel("Angular velocity (rad/s)")
    plt.title(name)
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("../Figures/" + name + ".png")
    plt.close()

# Three 5-second clips
plot_clip(5, 10, "motion_clip1")
plot_clip(20, 25, "motion_clip2")
plot_clip(40, 45, "motion_clip3")

print("Motion clips saved:")
print("  motion_clip1.png")
print("  motion_clip2.png")
print("  motion_clip3.png")
