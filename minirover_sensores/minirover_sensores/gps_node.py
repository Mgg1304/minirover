#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import serial
import re

class GPSCompassNode(Node):
    def __init__(self):
        super().__init__('gps_compass_node')
        
        # Configuración del puerto serial
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        
        # Publicadores
        self.gps_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.rumbo_pub = self.create_publisher(Float32, 'compass/heading', 10)
        self.vel_pub = self.create_publisher(Float32, 'gps/speed', 10)

        # Timer para leer serial
        self.timer = self.create_timer(1.0, self.read_serial)

    def read_serial(self):
        while self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            
            # Buscar coordenadas
            if line.startswith("Coordenadas:"):
                coords = re.findall(r"[-+]?\d*\.\d+|\d+", line)
                if len(coords) >= 2:
                    lat = float(coords[0])
                    lon = float(coords[1])
                    gps_msg = NavSatFix()
                    gps_msg.latitude = lat
                    gps_msg.longitude = lon
                    gps_msg.header.stamp = self.get_clock().now().to_msg()
                    self.gps_pub.publish(gps_msg)
                    self.get_logger().info(f"Publicando GPS: {lat}, {lon}")

            # Buscar velocidad
            elif line.startswith("Velocidad:"):
                speed_val = re.findall(r"[-+]?\d*\.\d+|\d+", line)
                if len(speed_val) >= 2:
                    speed_msg = Float32()
                    speed_msg.data = float(speed_val[0])
                    self.vel_pub.publish(speed_msg)
                    self.get_logger().info(f"Publicando velocidad: {speed_val[0]} km/h")

            # Buscar rumbo brújula
            elif line.startswith("Rumbo brújula:"):
                heading_val = re.findall(r"[-+]?\d*\.\d+|\d+", line)
                if heading_val:
                    heading_msg = Float32()
                    heading_msg.data = float(heading_val[0])
                    self.rumbo_pub.publish(heading_msg)
                    self.get_logger().info(f"Publicando rumbo: {heading_val[0]}°")

def main(args=None):
    rclpy.init(args=args)
    node = GPSCompassNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
