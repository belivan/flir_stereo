# FLIR Boson ROS Driver

This package provides a custom ROS 2 driver for the FLIR Boson camera. It can switch between raw and colorized video modes, with options for rectified video output.

## Installation

### 1. Install ROS 2

Ensure ROS 2 is installed on your system. Follow the [official ROS 2 installation guide](https://docs.ros.org/en/) for instructions specific to your ROS 2 distribution.

### 2. Teensy Setup if not already configured (for Time Synchronization of 2 or more cameras)

To make changes to the time synchronization behavior, set up your Teensy microcontroller:

1. **Install Required Software**

   - **Arduino IDE** or [Arduino CLI](https://arduino.github.io/arduino-cli/latest/).  
     - If you need a GUI and have a display available, the Arduino IDE may be the simplest.
     - For a headless system (e.g., SSH only, no display), use the Arduino CLI.
   - **Board Manager Package (Teensy)**  
     Add the following URL to Arduino/Arduino CLI preferences or board manager:  
     ```
     https://www.pjrc.com/teensy/package_teensy_index.json
     ```
   - **Udev Rules (optional on Linux)**  
     If you’re on Linux and need permission to access the Teensy USB port without root, copy the rules file:
     ```bash
     sudo cp 00-teensy.rules /etc/udev/rules.d/
     sudo udevadm control --reload-rules
     sudo udevadm trigger
     ```
   - **Teensy Loader**  
     - If you have a display, the Teensy Loader GUI can upload sketches directly.
     - On a headless system, install the [Teensy Loader Command Line](https://www.pjrc.com/teensy/loader_cli.html) utility for uploading .hex files.

2. **Reference Makefile and Scripts (optional and for future development only)**

   The Teensy setup includes a `Makefile` and other scripts (e.g., `ntp_time_sync`).  
   - These are primarily for reference from past development and might be adapted for automatic builds.  
   - If you need advanced time synchronization between the PC and Teensy in the future, these scripts can help streamline that process.

3. **Uploading Sketches or Firmware**

   - **time_sync_new**
     Uses PID control to synchronize the Teensy clock with GPS. Recommended for long-term accurate triggering where drift must remain minimal.
   - **pps_trigger**  
     A simpler script that tests sensor triggering without advanced time sync. It may experience clock drift over long periods but is sufficient for quick tests.

   **Headless/SSH Workflow (Example)**  
   1. **Build using Arduino CLI:**  
      ```bash
      arduino-cli compile --fqbn teensy:avr:teensy41 path/to/your_sketch --build-path path/to/desired_build_location
      ```
   2. **Upload using Teensy Loader Command Line:**  
      ```bash
      teensy_loader_cli --mcu=TEENSY41 -w path/to/your_sketch.ino.hex
      ```

   Replace `teensy41` with your actual Teensy model (e.g., `teensy36`, `teensy40`, etc.). This process allows you to compile and upload without a GUI or display.

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

