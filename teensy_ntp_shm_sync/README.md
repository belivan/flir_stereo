This repo contains a program to flash to the Teensy and a program to run on a computer. The Teensy will send packets containing time since boot in tenths of seconds over USB. The program running on the computer reads these packets and sends time information to chrony through shared memory.

The Teensy code is in ntp_time_sync/ and should be built with the Arduino editor. The program to run on the computer can be built using "make" and run using "sudo ./teensy_ntp_shm /dev/ttyACM0".

