#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from gps_interfaces.msg import GpsUtm
import serial
import pynmea2
import utm

class GpsSerialToUtm(Node):
    def __init__(self):
        super().__init__('gps_serial_to_utm')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 4800)
        self.declare_parameter('frame_id', 'GPS1_Frame')
        self.declare_parameter('topic', '/gps')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value

        self.pub = self.create_publisher(GpsUtm, self.topic, 10)

        self.get_logger().info(f"Opening serial port {self.port} @ {self.baud}...")
        self.ser = serial.Serial(self.port, self.baud, timeout=1.0)

        self.timer = self.create_timer(0.02, self.poll_serial)
        self.get_logger().info(f"Publishing {self.topic} (gps_interfaces/msg/GpsUtm)")

    def poll_serial(self):
        try:
            line = self.ser.readline()
            if not line:
                return

            s = line.decode('ascii', errors='ignore').strip()
            if not s.startswith('$'):
                return

            try:
                msg = pynmea2.parse(s, check=True)
            except Exception:
                return

            if msg.sentence_type != 'GGA':
                return
            if msg.latitude is None or msg.longitude is None:
                return

            lat = float(msg.latitude)
            lon = float(msg.longitude)

            alt = 0.0
            try:
                if msg.altitude not in (None, ''):
                    alt = float(msg.altitude)
            except Exception:
                alt = 0.0

            easting, northing, zone_num, zone_letter = utm.from_latlon(lat, lon)

            out = GpsUtm()
            out.header = Header()
            out.header.frame_id = self.frame_id

            # UTC + header stamp from GGA time (seconds since midnight)
            utc_str = str(msg.timestamp) if getattr(msg, 'timestamp', None) is not None else ""
            out.utc = utc_str

            sec = 0
            nsec = 0
            try:
                t = msg.timestamp
                sec = int(t.hour) * 3600 + int(t.minute) * 60 + int(t.second)
                nsec = int(getattr(t, "microsecond", 0)) * 1000
            except Exception:
                pass
            out.header.stamp.sec = sec
            out.header.stamp.nanosec = nsec

            out.latitude = lat
            out.longitude = lon
            out.altitude = alt

            try:
                out.hdop = float(msg.horizontal_dil) if msg.horizontal_dil not in (None, '') else 0.0
            except Exception:
                out.hdop = 0.0

            out.utm_easting = float(easting)
            out.utm_northing = float(northing)
            out.zone = int(zone_num)
            out.letter = str(zone_letter)

            self.pub.publish(out)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
        except Exception as e:
            self.get_logger().warn(f"Unexpected error: {e}")

def main():
    rclpy.init()
    node = GpsSerialToUtm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
