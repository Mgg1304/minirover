#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Intentar importar image_transport para soporte de compresión
try:
    from image_transport import ImageTransport
    IMAGE_TRANSPORT_AVAILABLE = True
except ImportError:
    IMAGE_TRANSPORT_AVAILABLE = False


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # ---------------- Configuración principal ----------------
        self.camera_index = 0       # Índice de la cámara USB
        self.fps = 30.0             # Frecuencia de publicación (frames por segundo)
        self.use_compression = False # True = imágenes comprimidas, False = normales
        # --------------------------------------------------------

        # Inicializa cámara
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'No se pudo abrir la cámara {self.camera_index}')
            return

        self.bridge = CvBridge()

        # Configura publicador según compresión
        if self.use_compression and IMAGE_TRANSPORT_AVAILABLE:
            self.it = ImageTransport(self)
            self.publisher_ = self.it.advertise('minirover/images', 10)
            self.get_logger().info('🗜️ Publicando imágenes comprimidas en minirover/images')
        else:
            self.publisher_ = self.create_publisher(Image, 'minirover/images', 10)
            self.get_logger().info('📷 Publicando imágenes sin compresión en minirover/images')

        # Timer para publicar imágenes según la frecuencia
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('No se pudo leer un frame de la cámara')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().warning('Frame publicado')

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
