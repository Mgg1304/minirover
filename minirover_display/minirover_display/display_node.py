#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import tkinter as tk
from PIL import Image, ImageTk
import cv2
import math

class DisplayNode(Node):
    def __init__(self, camera_index=None):
        super().__init__('display_node')
        
        self.data = {'GPS': None, 'Rumbo': 0.0, 'Velocidad': 0.0}

        # Ventana principal
        self.root = tk.Tk()
        self.root.title("Panel de Sensores")
        
        # Frames
        self.gps_frame = tk.Frame(self.root)
        self.gps_frame.pack(padx=10, pady=5)
        self.speed_frame = tk.Frame(self.root)
        self.speed_frame.pack(padx=10, pady=5)
        self.compass_frame = tk.Frame(self.root)
        self.compass_frame.pack(padx=10, pady=5)
        self.camera_frame = tk.Frame(self.root)
        self.camera_frame.pack(padx=10, pady=5)

        # Widgets
        self.gps_label = tk.Label(self.gps_frame, text="GPS: --", font=("Arial", 14))
        self.gps_label.pack()
        
        self.speed_label = tk.Label(self.speed_frame, text="Velocidad: 0 km/h", font=("Arial", 20), fg="blue")
        self.speed_label.pack()
        
        self.compass_canvas = tk.Canvas(self.compass_frame, width=200, height=200, bg='white')
        self.compass_canvas.pack()
        self.compass_center = (100, 100)
        self.compass_radius = 80

        # Cámara
        self.camera_index = camera_index
        if camera_index is not None:
            self.cap = cv2.VideoCapture(camera_index)
            self.camera_label = tk.Label(self.camera_frame)
            self.camera_label.pack()
        else:
            self.cap = None

        # Suscripciones ROS
        self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)
        self.create_subscription(Float32, 'compass/heading', self.heading_callback, 10)
        self.create_subscription(Float32, 'gps/speed', self.speed_callback, 10)
        
        # Timer para actualizar GUI
        self.timer = self.create_timer(0.1, self.update_window)

    # Callbacks ROS
    def gps_callback(self, msg):
        self.data['GPS'] = f"Lat: {msg.latitude:.6f}, Lon: {msg.longitude:.6f}"

    def heading_callback(self, msg):
        self.data['Rumbo'] = msg.data

    def speed_callback(self, msg):
        self.data['Velocidad'] = msg.data

    # Actualización de la ventana
    def update_window(self):
        # GPS
        if self.data['GPS']:
            self.gps_label.config(text=f"GPS: {self.data['GPS']}")

        # Velocidad
        self.speed_label.config(text=f"Velocidad: {self.data['Velocidad']:.2f} km/h")

        # Brújula
        self.compass_canvas.delete("all")
        self.compass_canvas.create_oval(
            self.compass_center[0]-self.compass_radius, self.compass_center[1]-self.compass_radius,
            self.compass_center[0]+self.compass_radius, self.compass_center[1]+self.compass_radius,
            outline='black', width=2
        )
        # Dibuja la aguja
        angle_rad = math.radians(self.data['Rumbo'])
        x = self.compass_center[0] + self.compass_radius * 0.8 * math.sin(angle_rad)
        y = self.compass_center[1] - self.compass_radius * 0.8 * math.cos(angle_rad)
        self.compass_canvas.create_line(
            self.compass_center[0], self.compass_center[1], x, y, fill='red', width=3
        )
        self.compass_canvas.create_text(self.compass_center, text="N", font=("Arial", 12, "bold"))

        # Cámara
        if self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame)
                imgtk = ImageTk.PhotoImage(image=img)
                self.camera_label.imgtk = imgtk
                self.camera_label.config(image=imgtk)

        self.root.update_idletasks()
        self.root.update()

def main(args=None):
    rclpy.init(args=args)
    node = DisplayNode(camera_index=0)  # Cambia a None si no quieres cámara
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap is not None:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
