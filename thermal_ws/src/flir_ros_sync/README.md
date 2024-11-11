# FLIR Boson ROS Driver

This package provides a custom ROS 2 driver for the FLIR Boson camera. It can switch between raw and colorized video modes, with options for rectified video output.

## Installation

### 1. Install ROS 2

Ensure ROS 2 is installed on your system. Follow the [official ROS 2 installation guide](https://docs.ros.org/en/) for instructions specific to your ROS 2 distribution.

### 2. Teensy Setup (for Time Synchronization of 2 or more cameras)

To make changes to the time synchronization behavior, set up your Teensy microcontroller:

1. Download and install the necessary software:
   - [Arduino IDE](https://www.arduino.cc/en/software)
   - Board manager package: `https://www.pjrc.com/teensy/package_teensy_index.json`
   - Udev rules (may be optional): 
      ```bash
      sudo cp 00-teensy.rules /etc/udev/rules.d/
      ```
   - Teensy Loader program (optional)

2. Note that the Teensy setup includes a `Makefile` along with other scripts, primarily for reference. This may be required if time synchronization between PC and Teensy is necessary in the future.

3. Upload sketches to the Teensy as needed. Options include:
   - **time_sync_new**: Uses PID control to synchronize the Teensy clock with GPS. Note: this may experience FPS issues (runs at 3Hz instead of 10Hz).
   - **pps_trigger**: A simpler script that tests sensor triggering without advanced time sync, which may lead to clock drift over long periods.

### 3. Package Build (ROS 2)

1. Install the required ROS 2 packages:

   ```bash
   sudo apt install ros-humble-image-transport ros-humble-cv-bridge ros-humble-camera-info-manager
   ```

2. Install **flirpy** for interfacing with the FLIR Boson cameras via a simple Python API:

   ```bash
   pip install flirpy
   ```

3. Ensure OpenCV is installed.

4. Install V4L2 headers for Linux:

   ```bash
   sudo apt install libv4l-dev
   ```

5. If needed, recompile the FSLP libraries (for FLIR Boson SDK):
   - Navigate to the Boson SDK folder and then the `FSLP_Files` subfolder.
   - Run `make` or `make all`, which attempts to build a 64-bit library based on your platform.

6. Copy udev rules to create symlinks such as _/dev/flir_boson_serial_#####_ that ultimately point to _/dev/ttyACM#_ for easier setup:

   ```bash
   sudo cp 42-flir-ros.rules /etc/udev/rules.d
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

7. Clone this package into your ROS workspace and build it:

   ```bash
   colcon build --packages-select flir_ros_sync
   ```

8. Source the ROS 2 workspace after building:

   ```bash
   source install/setup.bash
   ```

### 4. Review and Configure Launch Files

- Review the launch files located in `/flir_ros_sync/launch/yamaha_atv`:
   - **atv_both_cams**: Used for launching the stereo setup.
   - **test_single_cam**: Used for testing on a single camera.
   - Both launch files use **atv_single_cam** as the base configuration.
- Update `flir_id` to match your camera’s serial number and adjust other settings as needed for your project requirements.

## Usage (Single Camera)

1. Modify the `raw` parameter in the launch file (e.g., `flir_ros.launch`) to select between raw and colorized video.
2. Set the correct image size by adjusting the `width` and `height` parameters.
   - For a 640 FLIR camera, `width` should be set to 640 and `height` to 512.
3. Update the `camera_name` field in the launch file with a suitable camera name.
4. Save your camera's intrinsic parameters as a YAML file in `/flir_ros_sync/data/camera_info` and name it according to `camera_name` (see `/yamaha_atv/` for reference).
5. Launch the driver with:

   ```bash
   ros2 launch flir_ros_sync test_single_cam.launch.py
   ```

Thermal image data will be published to `/[camera]/image`, camera info will be published to `/[camera]/camera_info`, and rectified image data will be published to `/[camera]/image_rect`.

## Multiple Cameras

1. The udev rule that was installed creates unique symlinks for each FLIR camera based on its serial number, useful for multi-camera setups. You can list available serial numbers with:

   ```bash
   ls /dev/flir_boson*
   ```

2. To use multiple cameras, configure parameters and your cameras in a launch file as shown in the `atv_both_cams.launch.py`. Each camera will need its own node with unique `flir_id` and `camera_name` parameters.

3. Launch the driver with:

   ```bash
   ros2 launch flir_ros_sync atv_both_cams.launch.py
   ```

## Time Synchronization

If sync mode is set to **slave**, the Teensy triggers the cameras at 10 Hz. Otherwise, each camera generates its own 60 Hz pulse. Verify the `frame_rate` parameter in the launch file, which defaults to 10 Hz. This setting is important to ensure accurate time stamping, which is manually set at a 1/(frame_rate) interval.

## Notes

* The `udev` rules create symlinks `/dev/flir_boson_video` and `/dev/flir_boson_serial` for FLIR Boson cameras.
* The node publishes thermal images to `/[camera_name]/image` and camera info to `/[camera_name]/camera_info`.
* For multiple thermal cameras, adjust the `flir_id` and `camera_name` in each camera's node configuration.
* FLIR Boson cameras only require USB 2.0 ports.

## TODO

* Integrate the FLIR Boson SDK for NUC control, which is attempted in `trigger_ffc.py`.
* Determine time set offset per camera system.
* Add health metrics.
* Monitor and handle device disconnections.
