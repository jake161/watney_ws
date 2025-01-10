import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import argparse

class SerialNode(Node):
    def __init__(self, port, baudrate):
        super().__init__('serial_node')
        self.publisher_ = self.create_publisher(String, '/read', 10)
        self.subscription = self.create_subscription(String, '/write', self.write_serial, 10)
        self.subscription  # prevent unused variable warning
        self.ser = serial.Serial(port, baudrate, dsrdtr=None)
        self.ser.setRTS(False)
        self.ser.setDTR(False)
        self.serial_recv_thread = threading.Thread(target=self.read_serial)
        self.serial_recv_thread.daemon = True
        self.serial_recv_thread.start()

    def read_serial(self):
        while rclpy.ok():
            data = self.ser.readline().decode('utf-8')
            if data:
                self.get_logger().info(f"Received: {data}")
                msg = String()
                msg.data = data
                self.publisher_.publish(msg)

    def write_serial(self, msg):
        self.ser.write(msg.data.encode() + b'\n')

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Serial JSON Communication')
    parser.add_argument('port', type=str, help='Serial port name (e.g., COM1 or /dev/ttyUSB0)')
    parser.add_argument('baudrate', type=int, help='Serial baudrate (e.g., 9600 or 115200)')
    args = parser.parse_args()
    serial_node = SerialNode(args.port, args.baudrate)
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()