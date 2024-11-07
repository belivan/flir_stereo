#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import serial
import glob
import time

def find_teensy_port():
    # List all potential ports (update pattern if needed for your setup)
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    for port in ports:
        try:
            # Try to connect to each port to see if it's the Teensy
            ser = serial.Serial(port, 9600, timeout=1)  # Adjust baud rate if necessary
            time.sleep(2)  # Wait a moment for the connection to initialize
            ser.write(b'HELLO')  # Optional: Teensy might respond to a known command
            if ser.in_waiting > 0:
                ser.close()
                return port  # Return the port if we can connect successfully
            ser.close()
        except (OSError, serial.SerialException):
            pass
    return None

def teensy_serial_publisher():
    # Initialize the ROS node
    rospy.init_node('teensy_serial_publisher', anonymous=True)
    pub = rospy.Publisher('/teensy_data', String, queue_size=10)

    # Find and connect to the Teensy port
    serial_port = find_teensy_port()
    if serial_port is None:
        rospy.logerr("Teensy serial port not found!")
        return
    rospy.loginfo(f"Teensy connected on port: {serial_port}")
    
    ser = serial.Serial(serial_port, 115200, timeout=1)  # Open the serial port

    # Publish loop
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:  # Check if data is available
            line = ser.readline().decode('utf-8').strip()
            rospy.loginfo(f"Publishing: {line}")  # Log the output
            pub.publish(line)  # Publish to ROS topic
            
    ser.close()
    rospy.loginfo("Teensy serial connection closed.")

if __name__ == '__main__':
    try:
        teensy_serial_publisher()
    except rospy.ROSInterruptException:
        pass
