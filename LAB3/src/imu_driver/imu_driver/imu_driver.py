#!/usr/bin/env python3

import math
import serial

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from custom_interfaces.msg import IMUmsg


def valid_checksum(line: str) -> bool:
    line = line.strip()
    if not line.startswith('$') or '*' not in line:
        return False

    body = line[1:line.index('*')]
    given = line[line.index('*') + 1:].strip().upper()

    calc = 0
    for ch in body:
        calc ^= ord(ch)

    return f"{calc:02X}" == given


def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class IMUDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('frame_id', 'IMU1_Frame')

        port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value

        self.get_logger().info(f'Opening serial port: {port}')
        self.ser = serial.Serial(port, 115200, timeout=1.0)

        # Main publisher for custom message
        self.pub = self.create_publisher(IMUmsg, 'imu', 10)

        # Optional config command placeholder for 40 Hz output
        # Verify the exact register command with your VectorNav docs/emulator
        try:
            config_cmd = "$VNWRG,07,40*59\r\n"
            self.ser.write(config_cmd.encode('utf-8'))
            self.get_logger().info('Sent configuration command for 40 Hz output')
        except Exception as e:
            self.get_logger().warn(f'Could not send config command: {e}')

        self.timer = self.create_timer(0.001, self.read_serial)

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')
            return

        if not line:
            return

        if not line.startswith('$VNYMR'):
            return

        if not valid_checksum(line):
            self.get_logger().warn(f'Invalid checksum, skipping: {line}')
            return

        try:
            body = line[1:line.index('*')]
            fields = body.split(',')

            if len(fields) != 13:
                self.get_logger().warn(f'Unexpected field count: {len(fields)}')
                return

            if fields[0] != 'VNYMR':
                return

            yaw = float(fields[1])
            pitch = float(fields[2])
            roll = float(fields[3])

            mag_x = float(fields[4])
            mag_y = float(fields[5])
            mag_z = float(fields[6])

            acc_x = float(fields[7])
            acc_y = float(fields[8])
            acc_z = float(fields[9])

            gyro_x = float(fields[10])
            gyro_y = float(fields[11])
            gyro_z = float(fields[12])

            qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

            stamp = self.get_clock().now().to_msg()

            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = self.frame_id

            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw

            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z

            imu_msg.linear_acceleration.x = acc_x
            imu_msg.linear_acceleration.y = acc_y
            imu_msg.linear_acceleration.z = acc_z

            mag_msg = MagneticField()
            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = self.frame_id

            mag_msg.magnetic_field.x = mag_x
            mag_msg.magnetic_field.y = mag_y
            mag_msg.magnetic_field.z = mag_z

            msg = IMUmsg()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.imu = imu_msg
            msg.mag_field = mag_msg
            msg.raw = line

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Parse error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = IMUDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
