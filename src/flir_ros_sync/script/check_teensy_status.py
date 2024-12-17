#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import glob
import time
import sys


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
            self.get_logger().error("Teensy serial port not found! Shutting down node.")
            raise RuntimeError("Teensy serial port not found.")
        
        self.get_logger().info(f"Teensy connected on port: {serial_port}")

        try:
            self.ser = serial.Serial(serial_port, 115200, timeout=1)
            time.sleep(2)  # Wait for serial connection to stabilize
            self.get_logger().info(f"Opened serial port: {serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {serial_port}: {e}")
            raise e  # Re-raise exception to be handled in main

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
                    # self.get_logger().info(f"Published data: {data}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}")
            except UnicodeDecodeError as e:
                self.get_logger().error(f"Unicode decode error: {e}")

    def destroy_node_safe(self):
        """
        Safely destroys the node by closing the serial port and destroying the node.
        """
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                self.get_logger().info("Closed serial port.")
            except Exception as e:
                self.get_logger().error(f"Error closing serial port: {e}")
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TeensySerialPublisher()
    except (RuntimeError, serial.SerialException) as e:
        print(f"Initialization failed: {e}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Keyboard interrupt received. Shutting down node.")
    finally:
        if node:
            node.destroy_node_safe()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()