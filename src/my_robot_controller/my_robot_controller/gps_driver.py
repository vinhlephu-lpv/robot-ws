#!/usr/bin/env python3
"""
ROS 2 Hardware GPS Driver Node
Reads NMEA sentences ($GPGGA, $GPRMC) from Serial/USB GPS module (NEO-6M, NEO-M8N, GT-U7, RTK GPS)
and publishes standard ROS 2 sensor_msgs/msg/NavSatFix messages to /gps/fix.
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


def parse_nmea_coord(coord_str, direction):
    """
    Parses NMEA coordinate string (ddmm.mmmm or dddmm.mmmm) into decimal degrees.
    Example: '1046.5385', 'N' -> 10 + 46.5385/60 = 10.77564166
    """
    if not coord_str or not direction:
        return float('nan')
    try:
        dot_idx = coord_str.find('.')
        if dot_idx == -1:
            return float('nan')
        deg_len = dot_idx - 2
        deg = float(coord_str[:deg_len])
        minutes = float(coord_str[deg_len:])
        dec_deg = deg + (minutes / 60.0)
        if direction in ['S', 'W']:
            dec_deg = -dec_deg
        return dec_deg
    except Exception:
        return float('nan')


class GpsDriverNode(Node):
    def __init__(self):
        super().__init__('gps_driver_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('publish_topic', '/gps/fix')

        self.port_name = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_topic = self.get_parameter('publish_topic').value

        self.publisher_ = self.create_publisher(NavSatFix, self.publish_topic, 10)

        self.ser = None
        if not SERIAL_AVAILABLE:
            self.get_logger().error("pyserial package not installed! Please run 'pip install pyserial'.")
        else:
            try:
                self.ser = serial.Serial(self.port_name, self.baudrate, timeout=1.0)
                self.get_logger().info(f"Connected to GPS Serial port {self.port_name} at {self.baudrate} baud.")
            except Exception as e:
                self.get_logger().warn(f"Could not open serial port {self.port_name}: {e}. (Will retry in loop)")

        # Create timer loop (10 Hz)
        self.timer = self.create_timer(0.1, self.read_gps_data)

    def read_gps_data(self):
        if not SERIAL_AVAILABLE:
            return

        if self.ser is None or not self.ser.is_open:
            try:
                self.ser = serial.Serial(self.port_name, self.baudrate, timeout=1.0)
                self.get_logger().info(f"Reconnected to GPS Serial port {self.port_name}.")
            except Exception:
                return

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                    self.parse_gga(line)
        except Exception as e:
            self.get_logger().error(f"Error reading GPS serial port: {e}")

    def parse_gga(self, line):
        """
        Parses NMEA GGA Sentence:
        $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
        """
        parts = line.split(',')
        if len(parts) < 10:
            return

        raw_lat, lat_dir = parts[2], parts[3]
        raw_lon, lon_dir = parts[4], parts[5]
        fix_quality = parts[6]
        raw_alt = parts[9]

        lat = parse_nmea_coord(raw_lat, lat_dir)
        lon = parse_nmea_coord(raw_lon, lon_dir)
        try:
            alt = float(raw_alt) if raw_alt else 0.0
        except ValueError:
            alt = 0.0

        if math.isnan(lat) or math.isnan(lon):
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt

        try:
            qual = int(fix_quality)
            if qual > 0:
                msg.status.status = NavSatStatus.STATUS_FIX
            else:
                msg.status.status = NavSatStatus.STATUS_NO_FIX
        except ValueError:
            msg.status.status = NavSatStatus.STATUS_NO_FIX

        self.publisher_.publish(msg)
        self.get_logger().debug(f"[GPS Node] Published Fix: Lat={lat:.8f}, Lon={lon:.8f}, Alt={alt:.2f}m")


def main(args=None):
    rclpy.init(args=args)
    node = GpsDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
