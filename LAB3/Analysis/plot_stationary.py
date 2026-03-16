import csv
import math
import statistics
import matplotlib.pyplot as plt

time = []

ori_x, ori_y, ori_z, ori_w = [], [], [], []
gx, gy, gz = [], [], []
ax, ay, az = [], [], []
mx, my, mz = [], [], []

with open("stationary_data.csv", "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        time.append(float(row["time"]))

        ori_x.append(float(row["ori_x"]))
        ori_y.append(float(row["ori_y"]))
        ori_z.append(float(row["ori_z"]))
        ori_w.append(float(row["ori_w"]))

        gx.append(float(row["gyro_x"]))
        gy.append(float(row["gyro_y"]))
        gz.append(float(row["gyro_z"]))

        ax.append(float(row["acc_x"]))
        ay.append(float(row["acc_y"]))
        az.append(float(row["acc_z"]))

        mx.append(float(row["mag_x"]))
        my.append(float(row["mag_y"]))
        mz.append(float(row["mag_z"]))

roll, pitch, yaw = [], [], []

for x, y, z, w in zip(ori_x, ori_y, ori_z, ori_w):
    # roll
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    r = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    # pitch
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        p = math.degrees(math.copysign(math.pi / 2, sinp))
    else:
        p = math.degrees(math.asin(sinp))

    # yaw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    roll.append(r)
    pitch.append(p)
    yaw.append(yw)

def save_plot(x, ys, labels, title, ylabel, filename):
    plt.figure(figsize=(10, 5))
    for y, label in zip(ys, labels):
        plt.plot(x, y, label=label)
    plt.xlabel("Time (s)")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(filename, dpi=200)
    plt.close()

def save_hist(data, title, xlabel, filename):
    plt.figure(figsize=(8, 5))
    plt.hist(data, bins=50)
    plt.xlabel(xlabel)
    plt.ylabel("Count")
    plt.title(title)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(filename, dpi=200)
    plt.close()

save_plot(time, [gx, gy, gz], ["gyro_x", "gyro_y", "gyro_z"],
          "Gyroscope Time Series", "Angular Velocity", "gyro_timeseries.png")

save_plot(time, [ax, ay, az], ["acc_x", "acc_y", "acc_z"],
          "Accelerometer Time Series", "Linear Acceleration", "acc_timeseries.png")

save_plot(time, [mx, my, mz], ["mag_x", "mag_y", "mag_z"],
          "Magnetometer Time Series", "Magnetic Field", "mag_timeseries.png")

save_plot(time, [roll, pitch, yaw], ["roll", "pitch", "yaw"],
          "Orientation Time Series", "Angle (deg)", "orientation_timeseries.png")

save_hist(roll, "Roll Histogram", "Roll (deg)", "roll_hist.png")
save_hist(pitch, "Pitch Histogram", "Pitch (deg)", "pitch_hist.png")
save_hist(yaw, "Yaw Histogram", "Yaw (deg)", "yaw_hist.png")

print("Roll mean:", statistics.mean(roll))
print("Roll median:", statistics.median(roll))
print("Pitch mean:", statistics.mean(pitch))
print("Pitch median:", statistics.median(pitch))
print("Yaw mean:", statistics.mean(yaw))
print("Yaw median:", statistics.median(yaw))
