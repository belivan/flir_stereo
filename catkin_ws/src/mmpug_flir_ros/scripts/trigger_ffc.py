#!/usr/bin/env python3

import os
import rospy
from flirpy.camera.boson import Boson

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
    serial_list = rospy.get_param('serial_list', ["flir_boson_serial_322008", "flir_boson_serial_322011"])
    rospy.loginfo(f"Received serial list: {serial_list}")


    # Resolve the full paths for each serial port
    resolved_serial_ports = resolve_serial_ports(serial_list)

    if not resolved_serial_ports:
        rospy.logerr("No valid serial ports found, exiting...")
        return

    # Initialize cameras based on the resolved serial ports
    cameras = []
    for serial_port in resolved_serial_ports:
        try:
            camera = Boson(port=serial_port)
            cameras.append(camera)
            rospy.loginfo(f"Connected to camera on port: {serial_port}")
        except Exception as e:
            rospy.logerr(f"Failed to connect to camera on port {serial_port}: {e}")

    # Example function to trigger FFC for all connected cameras
    def trigger_ffc():
        rospy.loginfo("Triggering FFC for all cameras")
        for camera in cameras:
            try:
                camera.do_ffc()
                
                # Check if NUC table switch is desired
                if camera.get_nuc_desired() == 1:
                    rospy.loginfo(f"NUC Table Switch Desired for camera on port: {camera.port}. Switching NUC table.")
                    camera.do_nuc_table_switch()  # Perform the NUC table switch if needed
                    rospy.loginfo(f"NUC table updated for camera on port: {camera.port}")
            except Exception as e:
                rospy.logerr(f"Failed FFC or NUC on camera {camera.port}: {e}")

    # Set the loop rate to trigger FFC every 3 minutes (every 10 secs now)
    rate = rospy.Rate(1/10)  # 1/180 Hz = once every 180 seconds = 3 minutes
    first = True

    # Trigger FFC in an infinite loop
    while not rospy.is_shutdown():
        if first:
            first = False
        else:
            trigger_ffc()
        rate.sleep()

    # Close the cameras before exiting
    for camera in cameras:
        camera.close()

if __name__ == '__main__':
    main()
