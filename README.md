# flir_stereo
Thermal sensing integration with driver and time synchronization

Organization:

- catkin_ws/src
  Contains mmpug_common, mmpug_flir_ros, mmpug_r5timesync
  _common_ is a generic driver
  _flir_ros_ is the stereo driver
  _r5timesync_ is a time sync implementation for the wildfire team
- teensy_npt_shm_sync
  Contains time sync method with Teensy
