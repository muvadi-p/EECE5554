import csv
import math
import numpy as np
import matplotlib.pyplot as plt
import allantools

# Sampling rate from your lab
RATE = 40.0

# Arrays
gx, gy, gz = [], [], []
ax, ay, az = [], [], []

with open("stationary_data.csv", "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        gx.append(float(row["gyro_x"]))
        gy.append(float(row["gyro_y"]))
        gz.append(float(row["gyro_z"]))
        ax.append(float(row["acc_x"]))
        ay.append(float(row["acc_y"]))
        az.append(float(row["acc_z"]))

gx = np.array(gx)
gy = np.array(gy)
gz = np.array(gz)
ax = np.array(ax)
ay = np.array(ay)
az = np.array(az)

def extract_nbk(taus, adev):
    # Angle Random Walk / Velocity Random Walk proxy near tau=1
    idx_tau1 = np.argmin(np.abs(taus - 1.0))
    N = adev[idx_tau1]

    # Bias instability proxy = minimum Allan deviation
    idx_min = np.argmin(adev)
    B = adev[idx_min]
    tau_B = taus[idx_min]

    # Rate random walk proxy from largest tau
    K = adev[-1] / math.sqrt(taus[-1])

    return N, B, K, tau_B

def run_allan(data, name_prefix):
    taus_x, adev_x, _, _ = allantools.oadev(data[0], rate=RATE, taus='all')
    print(f"{name_prefix} X done")
    taus_y, adev_y, _, _ = allantools.oadev(data[1], rate=RATE, taus='all')
    print(f"{name_prefix} Y done")
    taus_z, adev_z, _, _ = allantools.oadev(data[2], rate=RATE, taus='all')
    print(f"{name_prefix} Z done")

    plt.figure(figsize=(10, 6))
    plt.loglog(taus_x, adev_x, label=f'{name_prefix} X')
    plt.loglog(taus_y, adev_y, label=f'{name_prefix} Y')
    plt.loglog(taus_z, adev_z, label=f'{name_prefix} Z')
    plt.xlabel('Averaging Time τ (s)')
    plt.ylabel('Allan Deviation')
    plt.title(f'{name_prefix} Allan Deviation')
    plt.legend()
    plt.grid(True, which='both')
    plt.tight_layout()
    outfile = f"{name_prefix.lower()}_allan.png"
    plt.savefig(outfile, dpi=200)
    plt.close()

    print(f"\n{name_prefix.upper()} PARAMETERS")
    for axis_name, taus, adev in [
        (f'{name_prefix} X', taus_x, adev_x),
        (f'{name_prefix} Y', taus_y, adev_y),
        (f'{name_prefix} Z', taus_z, adev_z),
    ]:
        N, B, K, tau_B = extract_nbk(taus, adev)
        print(f'{axis_name}:')
        print(f'  N (tau≈1 s): {N}')
        print(f'  B (minimum Allan deviation): {B}')
        print(f'  K (approximate): {K}')
        print(f'  tau at B: {tau_B}')

    print(f'\nSaved file: {outfile}')

print("Computing Allan deviation for gyroscope...")
run_allan((gx, gy, gz), "gyro")

print("\nComputing Allan deviation for accelerometer...")
run_allan((ax, ay, az), "acc")
