#!/usr/bin/env python3
import argparse
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from gps_driver.msg import GpsMsg
import utm

def dm_to_deg(dm: str, hemi: str) -> float:
    dot = dm.index(".")
    deg_len = dot - 2
    deg = float(dm[:deg_len])
    minutes = float(dm[deg_len:])
    val = deg + minutes / 60.0
    if hemi in ("S","W"):
        val = -val
    return val

def utc_to_stamp(utc_str: str):
    # hhmmss.sss -> seconds since midnight + nanosec
    utc = float(utc_str)
    hh = int(utc // 10000)
    mm = int((utc - hh*10000) // 100)
    ss = utc - hh*10000 - mm*100
    sec = int(hh*3600 + mm*60 + int(ss))
    nsec = int((ss - int(ss)) * 1e9)
    return sec, nsec, utc

class GPSDriver(Node):
    def __init__(self, port: str, baud: int = 4800):
        super().__init__("gps_driver")
        self.pub = self.create_publisher(GpsMsg, "/gps", 10)
        self.ser = serial.Serial(port, baudrate=baud, timeout=1)
        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info(f"Publishing /gps from {port} @ {baud}")

    def loop(self):
        line = self.ser.readline().decode("ascii", errors="ignore").strip()
        if not (line.startswith("$GPGGA") or line.startswith("$GNGGA")):
            return
        p = line.split(",")
        if len(p) < 10 or p[6] in ("", "0"):
            return

        try:
            stamp_sec, stamp_nsec, utc = utc_to_stamp(p[1])
            lat = dm_to_deg(p[2], p[3])
            lon = dm_to_deg(p[4], p[5])
            hdop = float(p[8]) if p[8] else float("nan")
            alt = float(p[9]) if p[9] else float("nan")
            e, n, zone, letter = utm.from_latlon(lat, lon)
        except Exception:
            return

        h = Header()
        h.frame_id = "GPS1_Frame"
        h.stamp.sec = stamp_sec
        h.stamp.nanosec = stamp_nsec

        m = GpsMsg()
        m.header = h
        m.latitude = lat
        m.longitude = lon
        m.altitude = alt
        m.hdop = hdop
        m.utm_easting = float(e)
        m.utm_northing = float(n)
        m.utc = float(utc)
        m.zone = str(zone)
        m.letter = str(letter)
        self.pub.publish(m)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=4800)
    a, _ = ap.parse_known_args()

    rclpy.init()
    node = GPSDriver(a.port, a.baud)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
