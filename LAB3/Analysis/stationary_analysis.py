import math
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

BAG_DIR = Path('../Data/rosbag2_2026_03_06-14_03_39')


def quaternion_to_euler_deg(x, y, z, w):
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)
    pitch = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


if not BAG_DIR.exists():
    raise FileNotFoundError(f'Bag directory not found: {BAG_DIR.resolve()}')

time_imu = []
time_mag = []

gyro_x, gyro_y, gyro_z = [], [], []
acc_x, acc_y, acc_z = [], [], []
mag_x, mag_y, mag_z = [], [], []

roll_deg, pitch_deg, yaw_deg = [], [], []

typestore = get_typestore(Stores.ROS2_HUMBLE)

with AnyReader([BAG_DIR], default_typestore=typestore) as reader:
    for connection, timestamp, rawdata in reader.messages():
        msg = reader.deserialize(rawdata, connection.msgtype)
        t_sec = timestamp * 1e-9

        if connection.topic == '/imu':
            time_imu.append(t_sec)

            gyro_x.append(msg.angular_velocity.x)
            gyro_y.append(msg.angular_velocity.y)
            gyro_z.append(msg.angular_velocity.z)

            acc_x.append(msg.linear_acceleration.x)
            acc_y.append(msg.linear_acceleration.y)
            acc_z.append(msg.linear_acceleration.z)

            r, p, y = quaternion_to_euler_deg(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            )
            roll_deg.append(r)
            pitch_deg.append(p)
            yaw_deg.append(y)

        elif connection.topic == '/magnetic':
            time_mag.append(t_sec)
            mag_x.append(msg.magnetic_field.x)
            mag_y.append(msg.magnetic_field.y)
            mag_z.append(msg.magnetic_field.z)

if len(time_imu) == 0:
    raise RuntimeError('No /imu messages found in the bag.')

if len(time_mag) == 0:
    raise RuntimeError('No /magnetic messages found in the bag.')

time_imu = np.array(time_imu)
time_mag = np.array(time_mag)

time_imu = time_imu - time_imu[0]
time_mag = time_mag - time_mag[0]

gyro_x = np.array(gyro_x)
gyro_y = np.array(gyro_y)
gyro_z = np.array(gyro_z)

acc_x = np.array(acc_x)
acc_y = np.array(acc_y)
acc_z = np.array(acc_z)

mag_x = np.array(mag_x)
mag_y = np.array(mag_y)
mag_z = np.array(mag_z)

roll_deg = np.array(roll_deg)
pitch_deg = np.array(pitch_deg)
yaw_deg = np.array(yaw_deg)

plt.figure(figsize=(10, 6))
plt.plot(time_imu, gyro_x, label='Gyro X')
plt.plot(time_imu, gyro_y, label='Gyro Y')
plt.plot(time_imu, gyro_z, label='Gyro Z')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('Gyroscope Time Series')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('gyro_timeseries.png')
plt.close()

plt.figure(figsize=(10, 6))
plt.plot(time_imu, acc_x, label='Accel X')
plt.plot(time_imu, acc_y, label='Accel Y')
plt.plot(time_imu, acc_z, label='Accel Z')
plt.xlabel('Time (s)')
plt.ylabel('Linear Acceleration (m/s^2)')
plt.title('Accelerometer Time Series')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('acc_timeseries.png')
plt.close()

plt.figure(figsize=(10, 6))
plt.plot(time_mag, mag_x, label='Mag X')
plt.plot(time_mag, mag_y, label='Mag Y')
plt.plot(time_mag, mag_z, label='Mag Z')
plt.xlabel('Time (s)')
plt.ylabel('Magnetic Field')
plt.title('Magnetometer Time Series')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('mag_timeseries.png')
plt.close()

plt.figure(figsize=(10, 6))
plt.plot(time_imu, roll_deg, label='Roll')
plt.plot(time_imu, pitch_deg, label='Pitch')
plt.plot(time_imu, yaw_deg, label='Yaw')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.title('Orientation Time Series')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('orientation_timeseries.png')
plt.close()

plt.figure(figsize=(10, 6))
plt.hist(roll_deg - np.median(roll_deg), bins=50, alpha=0.6, label='Roll')
plt.hist(pitch_deg - np.median(pitch_deg), bins=50, alpha=0.6, label='Pitch')
plt.hist(yaw_deg - np.median(yaw_deg), bins=50, alpha=0.6, label='Yaw')
plt.xlabel('Deviation from Median (deg)')
plt.ylabel('Count')
plt.title('Orientation Histogram Around Median')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('orientation_histogram.png')
plt.close()

print('Roll mean, median:', np.mean(roll_deg), np.median(roll_deg))
print('Pitch mean, median:', np.mean(pitch_deg), np.median(pitch_deg))
print('Yaw mean, median:', np.mean(yaw_deg), np.median(yaw_deg))
print('Saved files:')
print('  gyro_timeseries.png')
print('  acc_timeseries.png')
print('  mag_timeseries.png')
print('  orientation_timeseries.png')
print('  orientation_histogram.png')
