import csv
from pathlib import Path

import rclpy.serialization
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message

bag_dir = str(Path("../Data/prof_bag").resolve())
out_csv = "stationary_data.csv"

storage_options = StorageOptions(uri=bag_dir, storage_id="sqlite3")
converter_options = ConverterOptions(
    input_serialization_format="cdr",
    output_serialization_format="cdr"
)

reader = SequentialReader()
reader.open(storage_options, converter_options)

topic_types = reader.get_all_topics_and_types()
type_map = {topic.name: topic.type for topic in topic_types}

msg_type = get_message(type_map["/imu"])

with open(out_csv, "w", newline="") as f:
    writer = csv.writer(f)

    writer.writerow([
        "time",
        "ori_x","ori_y","ori_z","ori_w",
        "gyro_x","gyro_y","gyro_z",
        "acc_x","acc_y","acc_z",
        "mag_x","mag_y","mag_z"
    ])

    t0 = None

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic != "/imu":
            continue

        msg = rclpy.serialization.deserialize_message(data, msg_type)

        if t0 is None:
            t0 = timestamp

        t = (timestamp - t0) * 1e-9

        writer.writerow([
            t,
            msg.imu.orientation.x,
            msg.imu.orientation.y,
            msg.imu.orientation.z,
            msg.imu.orientation.w,
            msg.imu.angular_velocity.x,
            msg.imu.angular_velocity.y,
            msg.imu.angular_velocity.z,
            msg.imu.linear_acceleration.x,
            msg.imu.linear_acceleration.y,
            msg.imu.linear_acceleration.z,
            msg.mag_field.magnetic_field.x,
            msg.mag_field.magnetic_field.y,
            msg.mag_field.magnetic_field.z
        ])

print("CSV file written:", out_csv)
