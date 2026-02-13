import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class ArduinoNode(Node):
    """Subscribes to /detected_colors and forwards the colour string
    over serial to the Arduino, which lights the matching LEDs."""

    def __init__(self):
        super().__init__('arduino_node')

        # ---------- parameters ----------
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        self.ser = serial.Serial(port, baud, timeout=1)
        self.get_logger().info(f'Serial open on {port} @ {baud}')

        # ---------- subscriber ----------
        self.sub = self.create_subscription(
            String, 'detected_colors', self.color_callback, 10
        )

        # Keep track of last sent value to avoid flooding the serial link
        self._last_sent = None

    def color_callback(self, msg: String):
        data = msg.data  # e.g. "R", "GB", "RGB", "0"
        if data != self._last_sent:
            self._last_sent = data
            self.ser.write((data + '\n').encode())
            self.get_logger().info(f'Sent to Arduino: {data}')

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
