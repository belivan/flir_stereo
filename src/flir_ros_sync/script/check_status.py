#!/usr/bin/python3
import os
from Boson_SDK import *
import time
import rospy

if __name__ == '__main__':
    rospy.init_node('check_flir_status')
    rate = rospy.rate(10)
    myport = pyClient.Initialize(manualport="/dev/ttyACM0")

    while(not rospy.is_shutdown()):
        result, status = pyClient.bosonGetExtSyncMode()
        rospy.log_info("Client result: ", result)
        rospy.log_info("ext sync status: ", status)
        rate.sleep()


    pyClient.Close(myport)
