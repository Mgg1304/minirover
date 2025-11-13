#!/usr/bin/env python3
import io
#from PIL import Image, ImageTk
from sensor_msgs.msg import Image   
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Float32
import socket
import threading
import json
import time
from cv_bridge import CvBridge
import struct
import cv2


class NetworkBridge(Node):
    def __init__(self):
        super().__init__('network_bridge')

        # Crear la instancia de CvBridge
        self.bridge = CvBridge()
        
        # Publicador de comandos hacia ESP
        self.cmd_pub = self.create_publisher(String, 'ESP/cmd', 10)

        # Variables de telemetría
        self.gps_data = None
        self.rumbo = None
        self.velocidad = None

        # Suscripciones a los topics del GPS
        self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)
        self.create_subscription(Float32, 'compass/heading', self.rumbo_callback, 10)
        self.create_subscription(Float32, 'gps/speed', self.vel_callback, 10)
        self.create_subscription(Image, 'minirover/images', self.image_callback, 10)

        self.image_bytes = None

        # Hilos de red
        threading.Thread(target=self.command_server, daemon=True).start()
        threading.Thread(target=self.telemetry_server, daemon=True).start()

        self.get_logger().info("Nodo de red iniciado. Esperando conexiones del ordenador...")

    # --- Callbacks ROS ---
    def gps_callback(self, msg):
        self.gps_data = (msg.latitude, msg.longitude)

    def rumbo_callback(self, msg):
        self.rumbo = msg.data

    def vel_callback(self, msg):
        self.velocidad = msg.data

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Codificar a JPEG
            _, buffer = cv2.imencode('.jpg', cv_image)
            self.image_bytes = buffer.tobytes()
            self.get_logger().info("Imagen actualizada")
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')



    # --- Servidor para recibir comandos desde el PC ---
    def command_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(('0.0.0.0', 5000))
        server.listen(1)
        self.get_logger().info("Esperando conexión del PC en puerto 5000 (comandos)...")
        conn, addr = server.accept()
        self.get_logger().info(f"PC conectado desde {addr}")
        while True:
            data = conn.recv(1024)
            if not data:
                break
            cmd = data.decode().strip()
            if cmd:
                ros_msg = String()
                ros_msg.data = cmd
                self.cmd_pub.publish(ros_msg)
                self.get_logger().info(f"Comando recibido: {cmd}")

    # --- Servidor para enviar telemetría al PC ---
    def telemetry_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(('0.0.0.0', 6000))
        server.listen(1)
        self.get_logger().info("Esperando conexión del PC en puerto 6000 (telemetría)...")
        conn, addr = server.accept()
        self.get_logger().info(f"PC conectado para recibir telemetría desde {addr}")

        while True:
            telemetry = {
                "gps": self.gps_data if self.gps_data is not None else [0.0, 0.0],
                "rumbo": self.rumbo if self.rumbo is not None else 0.0,
                "velocidad": self.velocidad if self.velocidad is not None else 0.0
            }
            try:
                conn.sendall((json.dumps(telemetry) + "\n").encode())
                # Enviar imagen si existe
                if self.image_bytes:
                    # Primero enviar tamaño de la imagen (4 bytes)
                    conn.sendall(struct.pack(">I", len(self.image_bytes)))
                    # Luego enviar los bytes de la imagen
                    conn.sendall(self.image_bytes)
            except:
                break
            time.sleep(1/30)

def main(args=None):
    rclpy.init(args=args)
    node = NetworkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
