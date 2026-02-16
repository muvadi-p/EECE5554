#!/usr/bin/env python3
import argparse
import serial
import utm

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from gps_interfaces.msg import GpsUtm


def nmea_to_decimal(coord_str: str, hemi: str) -> float:
    """
    Convert NMEA coordinate to signed decimal degrees.
    lat: ddmm.mmmm (N/S)
    lon: dddmm.mmmm (E/W)
    """
    if not coord_str or not hemi:
        raise ValueError("Empty NMEA coordinate/hemisphere")

    hemi = hemi.strip().upper()

    if hemi in ("N", "S"):
        deg_len = 2
    elif hemi in ("E", "W"):
        deg_len = 3
    else:
        raise ValueError(f"Invalid hemisphere: {hemi}")

    deg = float(coord_str[:deg_len])
    minutes = float(coord_str[deg_len:])
    dec = deg + minutes / 60.0

    if hemi in ("S", "W"):
        dec = -dec

    return dec


class GPSDriver(Node):
    def __init__(self, port: str, baud: int):
        super().__init__("gps_driver")

        # Lab topic
        self.pub = self.create_publisher(GpsUtm, "/gps_utm", 10)

        # Serial
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=1.0
            )
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            raise

        self.get_logger().info(f"Opened serial port {port} @ {baud} baud")

        # Read periodically
        self.timer = self.create_timer(0.05, self.read_serial_once)  # ~20Hz

    def read_serial_once(self):
        try:
            line = self.ser.readline().decode(errors="ignore").strip()
            if not line:
                return

            # Only handle GGA (contains fix + lat/lon + hdop + altitude + UTC)
            # Example: $GPGGA,hhmmss.ss,lat,N,lon,W,fix,numsats,hdop,alt,M,...
            if "GGA" not in line:
                return
            if not line.startswith("$"):
                return

            parts = line.split(",")
            if len(parts) < 10:
                return

            # Required fields
            utc_str = parts[1].strip() if len(parts) > 1 else ""
            lat_raw = parts[2].strip() if len(parts) > 2 else ""
            lat_hemi = parts[3].strip() if len(parts) > 3 else ""
            lon_raw = parts[4].strip() if len(parts) > 4 else ""
            lon_hemi = parts[5].strip() if len(parts) > 5 else ""
            fix_quality = parts[6].strip() if len(parts) > 6 else ""
            hdop_str = parts[8].strip() if len(parts) > 8 else ""
            alt_str = parts[9].strip() if len(parts) > 9 else ""

            # Fix quality: "0" means invalid
            if fix_quality == "" or fix_quality == "0":
                return

            # Convert coordinates
            lat = nmea_to_decimal(lat_raw, lat_hemi)
            lon = nmea_to_decimal(lon_raw, lon_hemi)

            altitude = float(alt_str) if alt_str else 0.0
            hdop = float(hdop_str) if hdop_str else 0.0

            # Convert to UTM
            utm_easting, utm_northing, zone_number, zone_letter = utm.from_latlon(lat, lon)

            # Build message EXACTLY matching gps_interfaces/msg/GpsUtm
            msg = GpsUtm()

            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps"

            msg.latitude = float(lat)
            msg.longitude = float(lon)
            msg.altitude = float(altitude)
            msg.hdop = float(hdop)

            msg.utm_easting = float(utm_easting)
            msg.utm_northing = float(utm_northing)

            # UTC as string from GGA field 1 (hhmmss.ss). Keep as-is.
            msg.utc = utc_str

            msg.zone = int(zone_number)
            msg.letter = str(zone_letter)

            self.pub.publish(msg)

        except Exception as e:
            # Keep node alive even if one line is bad
            self.get_logger().warn(f"Read/parse/publish error: {e}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Serial port (e.g., /dev/ttyUSB0 or /dev/serial/by-id/...)")
    parser.add_argument("--baud", type=int, default=4800, help="Baud rate (default 4800)")
    args = parser.parse_args()

    rclpy.init()
    node = GPSDriver(args.port, args.baud)

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


if __name__ == "__main__":
    main()
