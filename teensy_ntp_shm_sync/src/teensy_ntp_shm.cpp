#include <sys/types.h>
#include <stdint.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <chrono>
#include <cerrno>
#include <cstring>

#include "teensy.h"
#include "ntp_shm.h"

int main(int argc, char** argv){
  if(argc < 2){
    std::cout << "Usage: ./teensy_npt_shm [teensy device path]" << std::endl;  // dev/ttyACM0
    return 0;
  }

  NTPSHM ntp_shm;

  Teensy teensy(argv[1]);
  TeensyPacket packet;
  bool first_packet = true;

  time_t initial_system_seconds = 0;
  time_t target_system_seconds = 0;
  int target_system_useconds = 0;
  time_t teensy_seconds_offset = 0;

  // This syncs your system clock with the teensy.
  // The teensy outputs time since boot in tenths of seconds. The initial teensy time used by this program is the first teensy time
  // rounded down to the nearest multiple of 10 seconds. This offset is subtracted from all future teensy times.
  // The initial system time used by this program is the current system time rounded down to the nearest multiple of 10 seconds.
  // The target system time is the initial system time plus the offset corrected teensy time.
  // This way when your system time is a multiple of 10 seconds the it will be synced with the teensy's LED which
  // lights up for 1 second every multiple of 10 seconds since boot.
  while(true){
    if(teensy.get_packet(&packet)){
      //std::cout << "packet: " << std::dec << packet.time_since_boot_in_tenths_of_seconds << std::endl;
      time_t teensy_seconds = packet.time_since_boot_in_tenths_of_seconds/10 - teensy_seconds_offset;
      int teensy_useconds = (packet.time_since_boot_in_tenths_of_seconds - packet.time_since_boot_in_tenths_of_seconds/10*10)*100000;

      time_t system_seconds;
      int system_useconds;
      get_system_time(&system_seconds, &system_useconds);
      
      if(first_packet){
        initial_system_seconds = system_seconds/10*10;
        teensy_seconds_offset = teensy_seconds/10*10;
        
        first_packet = false;
        continue;
      }

      target_system_seconds = initial_system_seconds + teensy_seconds;
      target_system_useconds = teensy_useconds;

      ntp_shm.add_time(target_system_seconds, target_system_useconds, system_seconds, system_useconds);

      if(teensy_useconds == 0){
        time_t offset = (system_seconds*1000000 + (time_t)system_useconds) - (target_system_seconds*1000000 + (time_t)target_system_useconds);
        std::cout << std::dec;
        std::cout << "system time: " << system_seconds << ", " << system_useconds << std::endl;
        std::cout << "teensy time: " << target_system_seconds << ", " << target_system_useconds << std::endl;
        std::cout << "offset: " << offset << " us" << std::endl << std::endl;
      }
    }
    
    
  }
  
  return 0;
}
