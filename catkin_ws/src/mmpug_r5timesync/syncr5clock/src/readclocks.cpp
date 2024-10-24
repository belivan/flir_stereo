/*
 * Read timer counts from physical addrss
 * Author: Sebastian Scherer, Henry Zhang (hengruiz@andrew.cmu.edu)
 */


#include <cerrno>
#include <fcntl.h> 
#include <cstring>
#include <termios.h>
#include <unistd.h>
#include <ctime>
#include <sys/timex.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <unistd.h>
#include <cmath>
#include <timesyncdata.h>

// Physical addresses of the timers
#define TSC_LO_ADDR 0x03010000
#define TSC_HI_ADDR 0x03010004
#define RTC_MSEC_ADDR 0x0c2a0010
#define RTC_SEC_ADDR 0x0c2a000c

#define OPENIVC

int map_addr(unsigned long *physical_addr, int *mem_fd, void **map, 
        volatile unsigned int **addr, size_t block_size) {
   
   printf("Mapping physical address 0x%lx\n", *physical_addr);
   // check if we have valid physical address
   if (!(*physical_addr)) {
       printf("Unspecified physical address!\n"); 
       exit(-1);
   }  
    
   // open /dev/mem
   *mem_fd = open("/dev/mem", O_RDONLY);
   if (mem_fd < 0) {
       printf("Failed to open /dev/mem :(\n");
       exit(-1);
   }

   // mmap target physical address to memory
   unsigned long MAP_MASK = (sysconf(_SC_PAGE_SIZE) - 1);
   *map = mmap(
            NULL,
            block_size,
            PROT_READ,
            MAP_PRIVATE,
            *mem_fd,
            (off_t)*physical_addr & ~MAP_MASK
            ); 
   if (*map ==  MAP_FAILED) {
        printf("mmap failed!\n");
        exit (-1);
   }

   *addr = (volatile unsigned int *) (*map + (*physical_addr & MAP_MASK));

   return 0;
}

volatile unsigned int *tsc_lo_addr;
volatile unsigned int *tsc_hi_addr;
unsigned long tsc_lo_count;
unsigned long tsc_hi_count;
/* Get times*/


int parse_chrony(double &frequency) {
    char cmd[] = "chronyc tracking |grep Frequency";    
    const int BUFSIZE=255;
    char buf[BUFSIZE];
    FILE *fp;
    frequency = 0.0;
    if ((fp = popen(cmd, "r")) == NULL) {
        printf("Error opening pipe!\n");
        return -1;
    }

    fscanf(fp,"Frequency       : %lf ppm %s",&frequency,&buf);


    if(buf[0]=='s')
      {
	frequency *=-1.0;
      }
    printf("%lf %s\n",frequency,buf);
    /*while (fgets(buf, BUFSIZE, fp) != NULL) {

      printf("OUTPUT: %s", buf);
      }*/

    if(pclose(fp))  {
        printf("Command not found or exited with error status\n");
        return -1;
    }

    return 0;
}

timespec diff(timespec start, timespec end)
{
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

void updateStructures(double &newfrequency, long int &newoffset, timespec &deltatime,timespec &deltatimemono)
{
  static double frequency = 31.25;
  static double chronyfreq = 0;
  static long int lastchrony_sec = 0;
  static long int inttimediff_usec = 0;
  timespec systemtime1,systemtime2,systemtime,monotonic,mono;
  //Get all the clock data with nothing in between
  clock_gettime(CLOCK_REALTIME, &systemtime1);
  clock_gettime(CLOCK_MONOTONIC_RAW, &monotonic);
  clock_gettime(CLOCK_MONOTONIC, &mono); 
  tsc_lo_count = (unsigned long)*tsc_lo_addr;
  tsc_hi_count = (unsigned long)*tsc_hi_addr;
  clock_gettime(CLOCK_REALTIME, &systemtime2);
  
  
  //Unfortunately it seems that adjtimex does not return the correct value. Using chronyc instead.
  // Since the call to chrony is a bit expensive. Let's only do that every 20s
  if(systemtime2.tv_sec - lastchrony_sec > 20)
    {
      lastchrony_sec = systemtime2.tv_sec;
      if(parse_chrony(chronyfreq) == -1)
	{
	  newoffset = inttimediff_usec;
	  return;
	}
    }

  //printf("tsc_lo_count:\t %lu\n", tsc_lo_count);
  //printf("tsc_hi_count:\t %lu\n", tsc_hi_count);
  double tsc_total = ((tsc_hi_count << 32) + tsc_lo_count);
  //printf("tsc_usec:\t %lu\n", tsc_total);
  //printf("freq: %ld\n", timexdata.freq);
  //Let's make live easy by just considering the same seconds for averaging:
  if(systemtime1.tv_sec == systemtime2.tv_sec)
    {
      systemtime.tv_sec = systemtime1.tv_sec;
      systemtime.tv_nsec   = (systemtime1.tv_nsec + systemtime2.tv_nsec)/2;
      //      printf("st1: %ld %ld\n",systemtime1.tv_sec,systemtime1.tv_nsec);
      // printf("sta: %ld %ld\n",systemtime.tv_sec,systemtime.tv_nsec);
      //printf("st2: %ld %ld\n",systemtime2.tv_sec,systemtime2.tv_nsec);

      
      //Calculate frequency
      //Don't use timex version for now. Doesn't seem to work.
      //double frequency =  31.25 * (1000000.0+(timexdata.freq/65536.0))/1000000.0;
      frequency =  31.25 * (1000000.0+chronyfreq)/1000000.0;
      double tsc_usec = tsc_total/frequency;
      double system_usec = (double)systemtime.tv_sec * 1000000.0 + (double)systemtime.tv_nsec / 1000.0;
      double timediff_usec = system_usec - tsc_usec;
      inttimediff_usec = (long int) (timediff_usec+0.5);

      //Delta of system time to raw time:
      timespec adjusted;
      unsigned int ticks_persecond = (unsigned int) (frequency * 1000000+0.5);
      double ratio = (31250000.0 / ((double)ticks_persecond));
      //printf("ratio: %f\n",ratio);
      TimeSync::convertRawTimeToFreq(ratio,monotonic,adjusted);
      deltatime = diff(adjusted,systemtime);

      deltatimemono = diff(mono,systemtime);

      //printf("Frequency: %f\n",frequency);
      //printf("tsc: %f\nsys: %f\n",tsc_usec, system_usec); 
      //printf("diff:%lu\n",inttimediff_usec);
    }
  newfrequency = frequency;
  newoffset = inttimediff_usec;
}

volatile sig_atomic_t stop;

void inthand(int signum) {
    stop = 1;
}


int main() {

#ifdef OPENIVC  
  char portname[] = "/sys/devices/aon_echo/data_channel";
  int datachannel = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (datachannel < 0)
    {
      printf ("Error %d opening %s: %s\n", errno, portname, strerror (errno));
      return -1;
    }
#endif
  
  unsigned int rtc_msec;
  unsigned int rtc_sec;
  unsigned long tsc_lo_physical = TSC_LO_ADDR;
  unsigned long tsc_hi_physical = TSC_HI_ADDR;
  int tsc_lo_mem_fd;
  int tsc_hi_mem_fd;   
  void *tsc_lo_map;
  void *tsc_hi_map; 
  size_t block_size = 4;     
  
  printf("Getting registers to oscillator\n");
  map_addr(&tsc_lo_physical, &tsc_lo_mem_fd, &tsc_lo_map, &tsc_lo_addr, block_size);
  map_addr(&tsc_hi_physical, &tsc_hi_mem_fd, &tsc_hi_map, &tsc_hi_addr, block_size);
  
  
  //Successfully passed to here:
  TimeSync::SharedMemoryInterface shminterface;
  TimeSync::Data shmdata;
  // combined tsc micro sec
  signal(SIGINT, inthand);
  while (!stop)
    {
      double frequency_mhz;
      long int offset_us;
      long int prioroffsetrawns = shmdata.offsetraw.tv_nsec;
      updateStructures(frequency_mhz, offset_us,shmdata.offsetraw,shmdata.offsetmono);
      shmdata.offsetraw.tv_nsec = (shmdata.offsetraw.tv_nsec + prioroffsetrawns)/2;
      //Convert to tickspersecond & only phase offset from second
      unsigned int ticks_persecond = (unsigned int) (frequency_mhz * 1000000+0.5);
      double doffset_us= offset_us;
      double seconds_us = 1000000.0*floor(doffset_us/1000000.0);
      doffset_us = doffset_us- seconds_us;
      unsigned int phaseoffset_ticks = (unsigned int)(frequency_mhz * doffset_us +0.5) ;
      shmdata.ticks_persecond = ticks_persecond;
      shmdata.offset_us = offset_us;
      shmdata.valid = 1;
      shmdata.frequency = frequency_mhz;
      shminterface.setData(shmdata);
      //printf("%f Mhz %lu usec\t ",frequency_mhz, offset_us);
      //printf("Freq: %u Phase: %u usec\n",ticks_persecond, phaseoffset_ticks);
      //printf("Monotonic RAW offset: %lu %lu \n",shmdata.offsetraw.tv_sec,shmdata.offsetraw.tv_nsec);
      //printf("Monotonic offset: %lu %lu \n",shmdata.offsetmono.tv_sec,shmdata.offsetmono.tv_nsec);
#ifdef OPENIVC  
      char buff[12] = "freq";
      memcpy(buff+4, &ticks_persecond, sizeof(ticks_persecond));
      memcpy(buff+8, &phaseoffset_ticks, sizeof(phaseoffset_ticks));
      write(datachannel, buff, sizeof(buff));
#endif
      //dprintf(datachannel,"freq%f %lu\n",frequency_mhz, offset_us);
      //Conversion function systemtime = offset_us + tsc_ticks/frequency_mhz
      usleep(500000);
    }
  #ifdef OPENIVC  
  close(datachannel);
  #endif
  printf("closing data channel\n");
  return 0;
}
