#include <ros/ros.h>
#include <vector>
#include "rawBoson.h"
#include <stdlib.h> // for char *realpath(const char *restrict path, char *restrict resolved_path); to read symlink for thermal serial
#include <stdio.h>
#include <iostream>
#include <ros/console.h>

#define CHECK_FATAL(x, err)    \
    if ((x))                   \
    {                          \
        ROS_FATAL_STREAM(err); \
        return 1;              \
    }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_sync_mode");

    ros::NodeHandle nhPrivate("~");

    int syncMode = 0; // default to disabled mode
    nhPrivate.getParam("sync_mode", syncMode);

    std::vector<std::string> serialList;
    nhPrivate.getParam("serial_list", serialList);

    if (serialList.empty())
    {
        ROS_ERROR("serial_list is empty!");
        return 1;
    }

    for (std::string serialPort : serialList)
    {
        char serialPortRoot[1024];
        char *serialResult = realpath(("/dev/" + serialPort).c_str(), serialPortRoot);
        CHECK_FATAL(!serialResult, "Serial port " << serialPort << " cannot be resolved!");
        set_sync_mode(syncMode, serialPortRoot);
        int returnedSyncMode = get_sync_mode(serialPortRoot);
        switch (returnedSyncMode)
        {
        case 0:
            ROS_INFO_STREAM(serialPort << " returns mode setting result: disabled.");
            break;

        case 1:
            ROS_INFO_STREAM(serialPort << " returns mode setting result: master.");
            break;

        case 2:
            ROS_INFO_STREAM(serialPort << " returns mode setting result: slave.");
            break;
        }
    }
}
