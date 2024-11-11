#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import glob
import time


def find_teensy_port():
    """
    Searches for the Teensy serial port by scanning common serial device patterns.
    Returns the first matching port found or None if no Teensy is detected.
    """
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
        self.ser = None  # Initialize serial attribute to None

        serial_port = find_teensy_port()
        if serial_port is None:
            self.get_logger().error("Teensy serial port not found!")
            return  # Exit __init__ but keep the node alive
        self.get_logger().info(f"Teensy connected on port: {serial_port}")

        try:
            self.ser = serial.Serial(serial_port, 115200, timeout=1)
            time.sleep(2)  # Wait for serial connection to stabilize
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {serial_port}: {e}")
            self.ser = None
            return

        self.timer = self.create_timer(0.1, self.publish_data)  # 10 Hz

    def publish_data(self):
        """
        Reads data from the Teensy serial port and publishes it to the '/teensy_data' topic.
        """
        if self.ser and self.ser.in_waiting > 0:
            try:
                data = self.ser.readline().decode('utf-8').rstrip()
                if data:
                    msg = String()
                    msg.data = data
                    self.pub.publish(msg)
                    self.get_logger().info(f"Published data: {data}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}")
            except UnicodeDecodeError as e:
                self.get_logger().error(f"Unicode decode error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TeensySerialPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TeensySerialPublisher node.")
    finally:
        if node.ser and node.ser.is_open:
            try:
                node.ser.close()
                node.get_logger().info("Closed serial port.")
            except Exception as e:
                node.get_logger().error(f"Error closing serial port: {e}")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
