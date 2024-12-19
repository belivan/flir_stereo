#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import UInt64
import serial
import glob
import time
import sys

HEADER = 0x55AA
FOOTER = 0x66BB
PACKET_SIZE = 8  # 2 bytes header + 4 bytes timestamp + 2 bytes footer

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
        self.pub = self.create_publisher(String, 'teensy/pps_data', 10)
        self.timestamp_pub = self.create_publisher(UInt64, 'teensy/trigger_timestamp', 10)
        self.ser = None

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

    def read_packet(self):
        """Attempt to read and validate a packet"""
        if self.ser.in_waiting < PACKET_SIZE:
            return None
            
        # Peek at first byte to check for header
        header_bytes = self.ser.read(2)
        if len(header_bytes) != 2:
            return None
            
        header = int.from_bytes(header_bytes, byteorder='little')
        if header != HEADER:
            # Not a valid packet, put bytes back
            self.ser.reset_input_buffer()
            return None
            
        # Read rest of packet
        packet_data = self.ser.read(PACKET_SIZE - 2)
        if len(packet_data) != PACKET_SIZE - 2:
            return None
            
        timestamp = int.from_bytes(packet_data[0:4], byteorder='little')
        footer = int.from_bytes(packet_data[4:6], byteorder='little')
        
        if footer != FOOTER:
            return None
            
        return timestamp
    
    def publish_data(self):
        """Reads and publishes both binary packets and debug messages"""
        if not self.ser or self.ser.in_waiting == 0:
            return

        try:
            # First try to read a complete packet
            timestamp = self.read_packet()
            if timestamp is not None:
                msg = UInt64()
                msg.data = timestamp
                self.timestamp_pub.publish(msg)
                self.get_logger().debug(f"Published timestamp: {timestamp}")

            # Read any remaining text data (debug messages)
            while self.ser.in_waiting > 0:
                try:
                    # Try to read a line of text
                    line = self.ser.readline()
                    if line:
                        try:
                            # Try to decode as text
                            text = line.decode('utf-8').rstrip()
                            if text:
                                msg = String()
                                msg.data = text
                                self.pub.publish(msg)
                        except UnicodeDecodeError:
                            # Not valid text, might be partial binary packet
                            pass
                except serial.SerialException as e:
                    self.get_logger().error(f"Serial read error: {e}")
                    break
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")

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