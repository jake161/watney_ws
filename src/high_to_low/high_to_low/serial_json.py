import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import argparse
import json

class SerialNode(Node):
    def __init__(self, port, baudrate):
        super().__init__('high_to_low_serial_node')
        self.publisher_ = self.create_publisher(String, '/h2l/read', 10) #continuosly publish to read
        self.subscription = self.create_subscription(String, '/h2l/write', self.write_serial, 10) # setup callback to write_serial
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
                self.handle_json(data)

    def handle_json(self, data):
        # parsing json for correct topic allocation
        try:
            json_data = json.loads(data)
            self.get_logger().info(f"Parsed JSON: {json_data}")
            msg = String()
            msg.data = json.dumps(json_data)
            self.publisher_.publish(msg)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON: {e}")

    def write_serial(self, msg):
        self.ser.write(msg.data.encode() + b'\n')

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Serial JSON Communication')
    parser.add_argument('port', type=str, nargs='?', default='/dev/serial0', help='Serial port name (e.g., COM1 or /dev/ttyUSB0)') # /dev/serial0 for RPi 4 UART pins
    parser.add_argument('baudrate', type=int, nargs='?', default=115200, help='Serial baudrate (e.g., 9600 or 115200)') # 115200 default baudrate for UGV 6x4
    args = parser.parse_args()
    serial_node = SerialNode(args.port, args.baudrate)
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()