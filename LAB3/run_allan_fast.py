import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from imu_interfaces.msg import IMUmsg
import allantools

bag_path = "/home/phillip/Downloads"

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

t = []
gx, gy, gz = [], [], []
ax, ay, az = [], [], []

count = 0
while reader.has_next():
    topic, data, timestamp = reader.read_next()

    if topic == "/imu":
        msg = deserialize_message(data, IMUmsg)

        t.append(timestamp * 1e-9)
        gx.append(msg.imu.angular_velocity.x)
        gy.append(msg.imu.angular_velocity.y)
        gz.append(msg.imu.angular_velocity.z)

        ax.append(msg.imu.linear_acceleration.x)
        ay.append(msg.imu.linear_acceleration.y)
        az.append(msg.imu.linear_acceleration.z)

        count += 1
        if count % 100000 == 0:
            print(f"Read {count} messages...")

t = np.array(t)
gx = np.array(gx, dtype=float)
gy = np.array(gy, dtype=float)
gz = np.array(gz, dtype=float)
ax = np.array(ax, dtype=float)
ay = np.array(ay, dtype=float)
az = np.array(az, dtype=float)

rate = 1 / np.mean(np.diff(t))
print("Samples:", len(t))
print("Sampling rate:", rate)

def allan_plot(data, title, filename):
    data = data - np.mean(data)

    taus, adev, _, _ = allantools.oadev(
        data,
        rate=rate,
        data_type="freq",
        taus="octave"
    )

    idx1 = np.argmin(np.abs(taus - 1.0))
    N = adev[idx1]

    idxb = np.argmin(adev)
    B = adev[idxb]

    idxk = min(idxb + 5, len(taus) - 1)
    K = adev[idxk] / np.sqrt(taus[idxk])

    plt.figure(figsize=(7, 5))
    plt.loglog(taus, adev, marker="o", label="ADEV")
    plt.loglog(taus, N / np.sqrt(taus), "r--", label=f"N={N:.2e}")
    plt.loglog(taus, K * np.sqrt(taus), "g--", label=f"K={K:.2e}")
    plt.loglog(taus, B * np.ones_like(taus), "b--", label=f"B={B:.2e}")

    plt.xlabel("Tau (s)")
    plt.ylabel("Allan deviation")
    plt.title(f"{title}\nN={N:.2e}, B={B:.2e}, K={K:.2e}")
    plt.grid(True, which="both")
    plt.legend()
    plt.savefig(filename, dpi=300, bbox_inches="tight")
    plt.close()

    return taus, adev
taus_gx, adev_gx = allan_plot(gx, "Gyro X", "gyro_x_allan.png")
print("Saved gyro_x_allan.png")
taus_gy, adev_gy = allan_plot(gy, "Gyro Y", "gyro_y_allan.png")
print("Saved gyro_y_allan.png")
taus_gz, adev_gz = allan_plot(gz, "Gyro Z", "gyro_z_allan.png")
print("Saved gyro_z_allan.png")

taus_ax, adev_ax = allan_plot(ax, "Accel X", "accel_x_allan.png")
print("Saved accel_x_allan.png")
taus_ay, adev_ay = allan_plot(ay, "Accel Y", "accel_y_allan.png")
print("Saved accel_y_allan.png")
taus_az, adev_az = allan_plot(az, "Accel Z", "accel_z_allan.png")
print("Saved accel_z_allan.png")

def params(taus, adev):
    idx1 = np.argmin(np.abs(taus - 1.0))
    N = adev[idx1]
    idxb = np.argmin(adev)
    B = adev[idxb]
    idxk = min(idxb + 5, len(taus) - 1)
    K = adev[idxk] / np.sqrt(taus[idxk])
    return N, B, K

print("Gyro X:", params(taus_gx, adev_gx))
print("Gyro Y:", params(taus_gy, adev_gy))
print("Gyro Z:", params(taus_gz, adev_gz))
print("Accel X:", params(taus_ax, adev_ax))
print("Accel Y:", params(taus_ay, adev_ay))
print("Accel Z:", params(taus_az, adev_az))
print("Plots saved in:", os.getcwd())
