#!/usr/bin/env python

import rospy
from flirpy.camera.boson import Boson
import time

# Function to trigger FFC for both cameras
def trigger_ffc(camera_left, camera_right):
    rospy.loginfo("Triggering FFC for both cameras")
    camera_left.do_ffc()
    camera_right.do_ffc()

def main():
    # Initialize the ROS node
    rospy.init_node('flir_ffc_trigger', anonymous=True)

    # Get the serial ports of the cameras from the launch file
    left_flir_serial = rospy.get_param('~left_flir_id')
    right_flir_serial = rospy.get_param('~right_flir_id')

    # Initialize the cameras
    rospy.loginfo(f"Connecting to left camera with serial: {left_flir_serial}")
    camera_left = Boson(port=left_flir_serial)

    rospy.loginfo(f"Connecting to right camera with serial: {right_flir_serial}")
    camera_right = Boson(port=right_flir_serial)

    # Set the loop rate to trigger FFC every 3 minutes
    rate = rospy.Rate(1/180)  # 1/180 Hz = once every 180 seconds = 3 minutes

    # Trigger FFC in an infinite loop
    while not rospy.is_shutdown():
        try:
            trigger_ffc(camera_left, camera_right)
            rate.sleep()
        except rospy.ROSInterruptException:
            break

    # Close the cameras before exiting
    camera_left.close()
    camera_right.close()

if __name__ == '__main__':
    main()
