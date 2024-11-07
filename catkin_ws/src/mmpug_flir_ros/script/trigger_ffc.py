#!/usr/bin/env python3

# import ctypes
# ctypes.cdll.LoadLibrary("/home/anton/Thermal_Camera/catkin_ws/src/mmpug_flir_ros/script/Boson_SDK/FSLP_Files/FSLP_64.so")

import os
import rospy
from flirpy.camera.boson import Boson
# import sys

# sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../script"))

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

def main():
    # Initialize the ROS node
    rospy.init_node('flir_ffc_trigger')

    rospy.loginfo("STARTING")

    # Get the list of serial ports from the launch file (for example, passed as a parameter)
    serial_list = rospy.get_param('~serial_list', ["flir_boson_serial_34582"])  # ["flir_boson_serial_322008", "flir_boson_serial_322011"])
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
            camera = CamAPI.pyClient(manualport=serial_port)
            cameras_boson_sdk[serial_port] = camera

            rospy.loginfo(f"Connected to camera on port: {serial_port}")
        except Exception as e:
            rospy.logerr(f"Failed to connect to camera on port {serial_port}: {e}")

    def trigger_ffc():
        rospy.loginfo("Triggering FFC for all cameras")
        for port, camera in cameras_flirpy.items():
            try:
                camera.do_ffc()
            except Exception as e:
                rospy.logerr(f"Failed FFC on camera {port}: {e}")
            
            try:
                # Check if NUC table switch is desired
                if camera.get_nuc_desired() == 1:
                    rospy.loginfo(f"NUC Table Switch Desired for camera on port: {port}. Switching NUC table.")
                    
                    camera.do_nuc_table_switch()  # Perform the NUC table switch if needed
                    result, nuc_type = cameras_boson_sdk[port].gaoGetNucType()
                    
                    rospy.loginfo(f"Result on {port}: {result} and Current NUC Type : {nuc_type}")
                else:
                    rospy.loginfo(f"NUC Table Switch NOT Desired for camera on port: {port}.")
                    result, nuc_type = cameras_boson_sdk[port].gaoGetNucType()
                    
                    rospy.loginfo(f"Result on {port}: {result} and Current NUC Type : {nuc_type}")

            except Exception as e:
                rospy.logerr(f"Failed NUC on camera {port}: {e}")

    # Set the loop rate to trigger FFC every 3 minutes (every 10 secs now)
    rate = rospy.Rate(1/180)  # 1/180 Hz = once every 180 seconds = 3 minutes
    first = True

    # Trigger FFC in an infinite loop
    while not rospy.is_shutdown():
        if first:
            first = False
        else:
            trigger_ffc()
        rate.sleep()

    # Close the cameras before exiting
    for port, camera in cameras_flirpy.items():
        camera.close()
    
    for port, camera in cameras_boson_sdk.items():
        camera.Close()

if __name__ == '__main__':
    main()
