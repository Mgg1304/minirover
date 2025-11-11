#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import serial.tools.list_ports


class ESPSerialNode(Node):
    def __init__(self):
        super().__init__('esp_serial_node')

        # Parámetros de conexión serial
        self.port = '/dev/ttyUSB0'
        self.baudrate = 115200
        self.ser = None

        self.get_logger().info('Iniciando nodo ESP Serial...')
        self.connect_serial()

        # Suscriptor al tópico ESP/cmd
        self.subscription = self.create_subscription(
            String,
            'ESP/cmd',
            self.listener_callback,
            10
        )

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f'Conectado al puerto serial {self.port} a {self.baudrate} bps')
        except serial.SerialException as e:
            self.get_logger().error(f'No se pudo abrir el puerto serial: {e}')
            self.ser = None

    def listener_callback(self, msg):
        if self.ser and self.ser.is_open:
            command = msg.data.strip().upper()
            valid_commands = {"FORWARD", "BACK", "LEFT", "RIGHT", "STOP"}

            if command in valid_commands:
                self.ser.write((command + '\n').encode('utf-8'))
                self.get_logger().info(f'Comando enviado al ESP: {command}')
            else:
                self.get_logger().warn(f'Comando no válido recibido: {command}')
        else:
            self.get_logger().error('Puerto serial no disponible, intentando reconectar...')
            self.connect_serial()


def main(args=None):
    rclpy.init(args=args)
    node = ESPSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido por el usuario.')
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
