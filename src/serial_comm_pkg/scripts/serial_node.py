#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import serial
import struct

class SerialNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('serial_node', anonymous=True)
        
        # Get parameters from the parameter server
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        baudrate = rospy.get_param('~baudrate', 9600)
        
        # Set up the serial connection
        self.ser = serial.Serial(port, baudrate, timeout=1)
        rospy.loginfo(f"Serial port opened on {port} with baudrate {baudrate}")
        
        # Publisher for IMU data
        self.imu_pub = rospy.Publisher('/imu_data', Imu, queue_size=10)
        
        # Subscriber to write to serial
        rospy.Subscriber('/write_serial', String, self.write_serial_callback)
        
        # Timer for reading from serial
        rospy.Timer(rospy.Duration(0.1), self.read_serial)

    def read_serial(self, event):
        if self.ser.in_waiting > 0:
            data = self.ser.read(56)  # Read 56 bytes (14 floats * 4 bytes each)
            rospy.loginfo(f"Read from serial: {data}")
            imu_msg = self.parse_imu_data(data)
            if imu_msg:
                self.imu_pub.publish(imu_msg)

    def parse_imu_data(self, data):
        try:
            # Unpack the binary data
            values = struct.unpack('ffffffffffffff', data)  # 14 floats: 4 for quaternion, 3 for accel, 3 for gyro, 3 for mag, 1 for temp
            if len(values) != 14:
                rospy.logwarn("Received data does not have 14 values")
                return None
            
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = 'imu_frame'
            imu_msg.orientation.x = round(values[0], 3)
            imu_msg.orientation.y = round(values[1], 3)
            imu_msg.orientation.z = round(values[2], 3)
            imu_msg.orientation.w = round(values[3], 3)
            imu_msg.linear_acceleration.x = round(values[4], 3)
            imu_msg.linear_acceleration.y = round(values[5], 3)
            imu_msg.linear_acceleration.z = round(values[6], 3)
            imu_msg.angular_velocity.x = round(values[7], 3)
            imu_msg.angular_velocity.y = round(values[8], 3)
            imu_msg.angular_velocity.z = round(values[9], 3)
            # Magnetic field and temperature are not part of the standard Imu message
            # You might need to create a custom message or use additional topics for them
            return imu_msg
        except struct.error as e:
            rospy.logwarn(f"Error parsing IMU data: {e}")
            return None

    def write_serial_callback(self, msg):
        rospy.loginfo(f"Writing to serial: {msg.data}")
        self.ser.write((msg.data + '\n').encode('utf-8'))

    def run(self):
        rospy.spin()
        self.ser.close()

if __name__ == '__main__':
    try:
        # Load parameters from the configuration file
        rospy.set_param('~port', '/dev/ttyUSB0')
        rospy.set_param('~baudrate', 9600)
        
        serial_node = SerialNode()
        serial_node.run()
    except rospy.ROSInterruptException:
        pass