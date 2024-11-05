#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import serial

class SerialNode:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600):
        # Initialize the node
        rospy.init_node('serial_node', anonymous=True)
        
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
            data = self.ser.readline().decode('utf-8').strip()
            rospy.loginfo(f"Read from serial: {data}")
            imu_msg = self.parse_imu_data(data)
            if imu_msg:
                self.imu_pub.publish(imu_msg)

    def parse_imu_data(self, data):
        try:
            values = list(map(float, data.split(',')))
            if len(values) != 13:
                rospy.logwarn("Received data does not have 13 values")
                return None
            
            imu_msg = Imu()
            imu_msg.orientation.x = values[0]
            imu_msg.orientation.y = values[1]
            imu_msg.orientation.z = values[2]
            imu_msg.linear_acceleration.x = values[3]
            imu_msg.linear_acceleration.y = values[4]
            imu_msg.linear_acceleration.z = values[5]
            imu_msg.angular_velocity.x = values[6]
            imu_msg.angular_velocity.y = values[7]
            imu_msg.angular_velocity.z = values[8]
            # Magnetic field and temperature are not part of the standard Imu message
            # You might need to create a custom message or use additional topics for them
            return imu_msg
        except ValueError as e:
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
        # Change the port and baudrate as necessary
        serial_node = SerialNode(port='/dev/ttyUSB0', baudrate=9600)
        serial_node.run()
    except rospy.ROSInterruptException:
        pass