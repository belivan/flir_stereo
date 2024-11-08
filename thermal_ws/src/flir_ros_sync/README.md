# FLIR Boson ROS Driver

This package provides a custom ROS driver for the FLIR Boson camera. Currently, it can switch between raw and colorized video.

This FLIR ROS driver also has time synchronization with Nvidia Jetson Xavier.

## Installation

1. Make sure you have OpenCV installed, with sources. I do a source build of OpenCV and haven't had any problems with that. You may wish to do the same. OpenCV is currently only used to do the color from YUV to RGB.
1. Clone this package into some workspace.
1. Copy the `udev` rules into place, as follows:

    ```bash
    sudo cp 42-flir-ros.rules /etc/udev/rules.d
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```

## Usage (single camera)

1. Select whether you want raw video (pre-AGC (automatic gain control)), or
   color video (post-AGC) by modifying the `raw` parameter in `flir_ros.launch`.
1. Put the image size in the `width` and `height` parameters in the launch file.
    For Flir 640, the width is 640 and the height is 512 for both raw and normal.
1. Put a reasonable camera name in the `camera_name` field in `flir_ros.launch`.
1. Put the path of the corresponding intrinsic parameter yaml file to the `intrinsic_url` field in `flir_ros.launch`. An example of the URL: `"package://flir_ros/camera_info/thermal_right_caminfo.yaml"`.
    See section 3.2 of
    [this](http://wiki.ros.org/camera_calibration_parsers#File_formats) link
    for yaml file formats.
1. Build the package with `catkin build -DCMAKE_BUILD_TYPE=Release`.
1. Launch the `flir_ros.launch` file.

Thermal image data will be published to `/[camera]/image`, camera info will be published to `/[camera]/camera_info` and the rectified image will be published to `/[camera]/image_rect`.

## Multiple cameras

The udev rule that you just installed will add additional symlinks for each flir camera based on its serial number. This gives a symlink that is unique to each camera, which is useful with multiple cameras. Examine the available serial numbers by running the following command:

```bash
ls /dev/flir_boson*
```

And pass in the appropriate path to the driver. Then, put reasonable ros parameters in the launch file for each camera just like `flir_ros.launch`. See `multiple_flir_ros.launch` as an example.

## Time Synchronization

This package assumes that the camera is being triggered by a 60 hz pulse that aligns with system time.

## Offline image rectification

1. For data without rectification and camera info, use the `launch/flir_pub_info.launch` to publish this information.
Change the bag file path in the launch file for different data.
1. Change the intrinsic parameter yaml file by changing the intrinsic_url` entry in the launch file.
1. Change the subscribing thermal image topic using the `thermal_topic` entry in the launch file.
1. Change the publishing thermal image camera info topic using the `camera_info_topic` entry in the launch file.
1. Change the frame id of the camera info frame using the `thermal_frame_id` entry in the launch file.

## Notes

* The `udev` rules will symlink all FLIR Boson cameras to `/dev/flir_boson_video` and `/dev/flir_boson_serial`.
* The node will always output thermal images to `/[camera_name]/image` and output camera info to `/[camera_name]/camera_info`.
* If you need to use multiple thermal cameras, you'll have to change the rule, as well as modify the launch file parameter `device_name`. Furthermore, you'll have to have one node per camera and have to assign individual camera names.
* The FLIR Boson camera only requires a USB 2.0 port.

## TODO

* Integrate the FLIR Boson API, or run the node separately.
* Output health metrics
* Check for disconnections
