import matplotlib
matplotlib.use("Agg")

import rosbag2_py
import numpy as np
import matplotlib.pyplot as plt
from imu_msgs.msg import IMUmsg
from rclpy.serialization import deserialize_message

bag_path = "/home/phillip/Downloads/moving_imu_0.db3"
storage_options = rosbag2_py.StorageOptions(
    uri=bag_path,
    storage_id="sqlite3"
)

converter_options = rosbag2_py.ConverterOptions(
    input_serialization_format="cdr",
    output_serialization_format="cdr"
)

reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

gyro_z = []
acc_x = []
acc_y = []
timestamps = []

while reader.has_next():
    topic, data, t = reader.read_next()

    if topic == "/imu":
        msg = deserialize_message(data, IMUmsg)

        timestamps.append(t * 1e-9)
        gyro_z.append(msg.imu.angular_velocity.z)
        acc_x.append(msg.imu.linear_acceleration.x)
        acc_y.append(msg.imu.linear_acceleration.y)

timestamps = np.array(timestamps)
gyro_z = np.array(gyro_z)
acc_x = np.array(acc_x)
acc_y = np.array(acc_y)

t = timestamps - timestamps[0]

clips = [(20, 25), (60, 65), (120, 125)]

for i, (start, end) in enumerate(clips, start=1):
    mask = (t >= start) & (t <= end)

    plt.figure(figsize=(8, 4))
    plt.plot(t[mask], gyro_z[mask], label="Gyro Z")
    plt.plot(t[mask], acc_x[mask], label="Accel X")
    plt.plot(t[mask], acc_y[mask], label="Accel Y")

    plt.xlabel("Time (seconds)") 
    plt.ylabel("Acceleration (m/s^2) and Angular Velocity (rad/s)")
   
    plt.title(f"IMU Motion Clip {i}")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f"motion_clip_{i}.png", dpi=300)
    plt.close()

print("Motion clips saved.")
