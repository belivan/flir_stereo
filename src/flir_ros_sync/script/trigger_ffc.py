#!/usr/bin/env python3

import os
import rospy
from flirpy.camera.boson import Boson

from Boson_SDK import *

def resolve_serial_ports(serial_list):
    """
    Resolve each serial port to its full path using realpath and ensure it exists.
    """
    resolved_serial_ports = []
    for serial_port in serial_list:
        try:
            # Resolve the full path
            serial_port_root = os.path.realpath(f"/dev/{serial_port}")

            if not os.path.exists(serial_port_root):
                raise FileNotFoundError(f"Serial port {serial_port_root} cannot be resolved!")

            rospy.loginfo(f"Resolved serial port: {serial_port} to {serial_port_root}")
            resolved_serial_ports.append(serial_port_root)
        except Exception as e:
            rospy.logerr(f"Error resolving serial port {serial_port}: {e}")

    return resolved_serial_ports

def trigger_ffc(cameras_flirpy, cameras_boson_sdk):
    rospy.loginfo("Triggering FFC for all cameras")
    for port, camera in cameras_flirpy.items():
        # FLIRpy FFC trigger
        try:
            state = camera.get_ffc_state()
            rospy.loginfo(f"FFC State on {port}: {state}")
            camera.do_ffc()
            rospy.loginfo(f"Triggered FFC for camera on port: {port}")
        except Exception as e:
            rospy.logerr(f"Failed to trigger FFC on camera {port}: {e}")

        # NUC table switch (not working yet)
        try:
            rospy.loginfo(f"NUC Table Switch Desired for camera on port: {port}. Switching NUC table.")
            camera.do_nuc_table_switch()  # Perform the NUC table switch if needed
            result, nuc_type = cameras_boson_sdk[port].gaoGetNucType()
            rospy.loginfo(f"Result on {port}: {result} and Current NUC Type : {nuc_type}")
        except Exception as e:
            rospy.logerr(f"Failed NUC on camera {port}: {e}")

def cleanup(cameras_flirpy, cameras_boson_sdk):
    rospy.loginfo("Shutting down: Closing camera connections.")
    for port, camera in cameras_flirpy.items():
        try:
            camera.close()
            rospy.loginfo(f"Closed camera on port: {port}")
        except Exception as e:
            rospy.logerr(f"Failed to close camera on port {port}: {e}")

    for port, camera in cameras_boson_sdk.items():
        try:
            camera.Close()
            rospy.loginfo(f"Closed Boson SDK camera on port: {port}")
        except Exception as e:
            rospy.logerr(f"Failed to close Boson SDK camera on port {port}: {e}")

def main():
    # Initialize the ROS node
    rospy.init_node('flir_ffc_trigger')

    rospy.loginfo("STARTING")

    # Get the list of serial ports from the launch file
    serial_list = rospy.get_param('~serial_list', ["flir_boson_serial_34582"])
    rospy.loginfo(f"Received serial list: {serial_list}")

    # Resolve the full paths for each serial port
    resolved_serial_ports = resolve_serial_ports(serial_list)

    if not resolved_serial_ports:
        rospy.logerr("No valid serial ports found, exiting...")
        return

    # Initialize cameras based on the resolved serial ports
    cameras_flirpy = {}
    cameras_boson_sdk = {}
    for serial_port in resolved_serial_ports:
        try:
            camera = Boson(port=serial_port)
            cameras_flirpy[serial_port] = camera

            # Initialize the camera using the Boson SDK
            camera_sdk = CamAPI.pyClient(manualport=serial_port)
            cameras_boson_sdk[serial_port] = camera_sdk

            rospy.loginfo(f"Connected to camera on port: {serial_port}")
        except Exception as e:
            rospy.logerr(f"Failed to connect to camera on port {serial_port}: {e}")

    # Register shutdown callback
    rospy.on_shutdown(lambda: cleanup(cameras_flirpy, cameras_boson_sdk))

    # Set up a ROS Timer to trigger FFC every 3 minutes
    timer = rospy.Timer(rospy.Duration(180), lambda event: trigger_ffc(cameras_flirpy, cameras_boson_sdk))

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
