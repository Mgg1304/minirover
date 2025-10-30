import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('minirover_control')
        self.publisher = self.create_publisher(String, 'arduino/cmd', 10)
        self.get_logger().info("Nodo de control por teclado iniciado. Usa flechas ↑↓←→ para controlar el dron.")

        # Listener del teclado
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()

    def on_press(self, key):
        try:
            msg = String()
            if key == keyboard.Key.up:
                msg.data = "FORWARD"
            elif key == keyboard.Key.down:
                msg.data = "BACKWARD"
            elif key == keyboard.Key.left:
                msg.data = "LEFT"
            elif key == keyboard.Key.right:
                msg.data = "RIGHT"
            else:
                return  # Ignorar otras teclas

            self.publisher.publish(msg)
            self.get_logger().info(f"Enviado: {msg.data}")

        except Exception as e:
            self.get_logger().error(f"Error al leer teclado: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
