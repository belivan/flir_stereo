#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
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
            ser = serial.Serial(port, 115200, timeout=1)  # Adjust baud rate if necessary
            time.sleep(2)  # Wait a moment for the connection to initialize

            if ser.in_waiting > 0:
                ser.close()
                return port  # Return the port if we can connect successfully
            ser.close()
        except (OSError, serial.SerialException):
            pass
    return None

class TeensySerialPublisher(Node):
    def __init__(self):
        super().__init__('teensy_serial_publisher')
        self.pub = self.create_publisher(String, '/teensy_data', 10)
        serial_port = find_teensy_port()
        if serial_port is None:
            self.get_logger().error("Teensy serial port not found!")
            return
        self.get_logger().info(f"Teensy connected on port: {serial_port}")
        self.ser = serial.Serial(serial_port, 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.publish_data)

    def publish_data(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            self.get_logger().info(f"Publishing: {line}")
            msg = String()
            msg.data = line
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeensySerialPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.get_logger().info("Teensy serial connection closed.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
