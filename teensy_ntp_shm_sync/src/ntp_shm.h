#ifndef _NTP_SHM_H_
#define _NTP_SHM_H_

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


struct shmTime
{
  int mode;/* 0 - if valid set
	    *       use values, 
	    *       clear valid
	    * 1 - if valid set 
	    *       if count before and after read of values is equal,
	    *         use values 
	    *       clear valid
	    */
  int count;
  time_t clockTimeStampSec;
  int clockTimeStampUSec;
  time_t receiveTimeStampSec;
  int receiveTimeStampUSec;
  int leap;
  int precision;
  int nsamples;
  int valid;
  int pad[10];
};

class NTPSHM {
private:
  static const unsigned int NTPD_BASE = 0x4e545030; // this is the key that chrony looks for in shared memory
  static const unsigned int SHM_UNIT  = 0;
  volatile struct shmTime* shm_time; // gpsd makes this volatile to try to prevent the compiler from reordering writes
  int shmid;

public:
  NTPSHM();
  void add_time(time_t seconds, int useconds, time_t system_time_seconds, int system_time_useconds);
  void print_shm();
};

void get_system_time(time_t* seconds, int* useconds);

#endif
