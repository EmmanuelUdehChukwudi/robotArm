import rclpy
from rclpy.node import Node
import serial

class SerialMonitorNode(Node):
    def __init__(self):
        super().__init__('serial_monitor_node')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f'Successfully connected to {port} at {baudrate} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {port} at {baudrate} baud: {str(e)}')
            rclpy.shutdown()
            return

        self.timer = self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        if self.serial_port.in_waiting > 0:
            data = self.serial_port.readline().decode('ascii', errors='replace').strip()
            self.get_logger().info(f'Received: {data}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
