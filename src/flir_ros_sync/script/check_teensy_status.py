#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
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
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    for port in ports:
        try:
            ser = serial.Serial(port, 115200, timeout=1)  # Adjust baud rate if necessary
            time.sleep(2)  # Wait a moment for the connection to initialize

            if ser.in_waiting > 0:
                ser.close()
                return port
            ser.close()
        except (OSError, serial.SerialException):
            pass
    return None


class TeensySerialPublisher(Node):
    def __init__(self):
        super().__init__('teensy_serial_publisher')
        self.timestamp_pub = self.create_publisher(UInt64, 'teensy/trigger_timestamp', 10)
        self.ser = None
        self.buffer = bytearray()

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
            raise e

        self.timer = self.create_timer(0.10, self.publish_data)  # 10 Hz

    def read_packet(self):
        """Attempts to extract one complete packet from the buffer."""
        # Read all available bytes into buffer
        if self.ser.in_waiting > 0:
            self.buffer.extend(self.ser.read(self.ser.in_waiting))

        # Search for a valid packet in the buffer
        i = 0
        while i <= len(self.buffer) - PACKET_SIZE:
            header = int.from_bytes(self.buffer[i:i+2], 'little')
            if header == HEADER:
                timestamp = int.from_bytes(self.buffer[i+2:i+6], 'little')
                footer = int.from_bytes(self.buffer[i+6:i+8], 'little')
                if footer == FOOTER:
                    # Valid packet found
                    del self.buffer[i:i+8]
                    return timestamp
                else:
                    # If footer doesn't match, move ahead and keep searching
                    i += 2
            else:
                i += 1
        return None

    def publish_data(self):
        if not self.ser:
            return

        try:
            # Extract and publish all available packets
            # It's possible that multiple packets have arrived since last call
            while True:
                timestamp = self.read_packet()
                if timestamp is None:
                    break
                msg = UInt64()
                msg.data = timestamp
                self.timestamp_pub.publish(msg)
                # self.get_logger().info(f"Published timestamp: {timestamp}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")

    def destroy_node_safe(self):
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
