#include "ntp_shm.h"

NTPSHM::NTPSHM(){
  unsigned int perms = 0600; // root level permissions

  // init shared memory
  shmid = shmget((key_t)(NTPD_BASE + SHM_UNIT), sizeof(struct shmTime), (int)(IPC_CREAT | perms));
  if(shmid == -1){
    std::cout << "shmget failed. Make sure you are running as root." << std::endl;
    exit(1);
  }
  
  shm_time = (struct shmTime*)shmat(shmid, 0, 0);
  if((int)(long)shm_time == -1){
    std::cout << "shmat failed." << std::endl;
    exit(1);
  }
}

void NTPSHM::add_time(time_t seconds, int useconds, time_t system_time_seconds, int system_time_useconds){
  shm_time->valid = 0;
  shm_time->mode = 1;
  shm_time->nsamples = 3; // gpsd sets this to 3
  shm_time->count += 1;

  asm volatile ("" : : : "memory"); // memory barrier, ROS ntpd_driver does this

  shm_time->clockTimeStampSec = seconds;
  shm_time->clockTimeStampUSec = useconds;
  shm_time->receiveTimeStampSec = system_time_seconds;
  shm_time->receiveTimeStampUSec = system_time_useconds;

  shm_time->leap = 0;
  shm_time->precision = -1;
  
  asm volatile ("" : : : "memory"); // memory barrier

  shm_time->count += 1;
  shm_time->valid = 1;
}

void NTPSHM::print_shm(){
  std::cout << "mode: " << shm_time->mode << std::endl;
  std::cout << "count: " << shm_time->count << std::endl;
  std::cout << "clockTimeStampSec: " << shm_time->clockTimeStampSec << std::endl;
  std::cout << "clockTimeStampUSec: " << shm_time->clockTimeStampUSec << std::endl;
  std::cout << "receiveTimeStampSec: " << shm_time->receiveTimeStampSec << std::endl;
  std::cout << "receiveTimeStampUSec: " << shm_time->receiveTimeStampUSec << std::endl;
  std::cout << "precision: " << shm_time->precision << std::endl;
  std::cout << "nsamples: " << shm_time->nsamples << std::endl;
  std::cout << "valid: " << shm_time->valid<< std::endl;
}


void get_system_time(time_t* seconds, int* useconds){
  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
  std::chrono::system_clock::duration duration = now.time_since_epoch();

  *seconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
  *useconds = std::chrono::duration_cast<std::chrono::microseconds>(duration).count() - *seconds*1000000;
}
