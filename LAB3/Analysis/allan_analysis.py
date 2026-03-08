from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import allantools
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

BAG_FILE = Path('/home/phillip/Downloads/imu_bag_2_0.mcap')

if not BAG_FILE.exists():
    raise FileNotFoundError(f'Long bag not found: {BAG_FILE}')

acc_x, acc_y, acc_z = [], [], []

typestore = get_typestore(Stores.ROS2_HUMBLE)

print(f'Reading bag: {BAG_FILE}')

with AnyReader([BAG_FILE], default_typestore=typestore) as reader:
    for connection, timestamp, rawdata in reader.messages():
        msg = reader.deserialize(rawdata, connection.msgtype)

        if hasattr(msg, 'imu'):
            acc_x.append(msg.imu.linear_acceleration.x)
            acc_y.append(msg.imu.linear_acceleration.y)
            acc_z.append(msg.imu.linear_acceleration.z)

acc_x = np.array(acc_x)
acc_y = np.array(acc_y)
acc_z = np.array(acc_z)

if len(acc_x) == 0:
    raise RuntimeError('No accelerometer data extracted from long bag.')

print(f'Samples loaded: {len(acc_x)}')

rate = 40.0

def extract_nbk(taus, adev):
    idx_tau1 = np.argmin(np.abs(taus - 1.0))
    N = adev[idx_tau1]

    idx_min = np.argmin(adev)
    B = adev[idx_min]
    tau_B = taus[idx_min]

    K = adev[-1] / np.sqrt(taus[-1])

    return N, B, K, tau_B

print('Computing Allan deviation for accelerometer...')

taus_ax, adev_ax, _, _ = allantools.oadev(acc_x, rate=rate, taus='all')
print('Accel X done')

taus_ay, adev_ay, _, _ = allantools.oadev(acc_y, rate=rate, taus='all')
print('Accel Y done')

taus_az, adev_az, _, _ = allantools.oadev(acc_z, rate=rate, taus='all')
print('Accel Z done')

plt.figure(figsize=(10, 6))
plt.loglog(taus_ax, adev_ax, label='Accel X')
plt.loglog(taus_ay, adev_ay, label='Accel Y')
plt.loglog(taus_az, adev_az, label='Accel Z')
plt.xlabel('Averaging Time τ (s)')
plt.ylabel('Allan Deviation')
plt.title('Accelerometer Allan Deviation')
plt.legend()
plt.grid(True, which='both')
plt.tight_layout()
plt.savefig('acc_allan.png')
plt.close()

print('\nACCELEROMETER PARAMETERS')
for axis_name, taus, adev in [
    ('Accel X', taus_ax, adev_ax),
    ('Accel Y', taus_ay, adev_ay),
    ('Accel Z', taus_az, adev_az),
]:
    N, B, K, tau_B = extract_nbk(taus, adev)
    print(f'{axis_name}:')
    print(f'  N (at tau = 1 s): {N}')
    print(f'  B (minimum Allan deviation): {B}')
    print(f'  K (approximate): {K}')
    print(f'  tau at B: {tau_B}')

print('\nSaved file: acc_allan.png')
