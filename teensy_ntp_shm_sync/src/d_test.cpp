#include <stdint.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>

int main(int argc, char** argv){
  int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
  
  // If the device has been opened successfully, configure it
  if(fd != -1) {
    // set non blocking
    fcntl(fd, F_SETFL, FNDELAY);
    
    struct termios options;
    tcgetattr(fd, &options);
    
    // set baud rate
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    /*
    // ignore modem lines and enable receiver
    options.c_cflag |= (CLOCAL | CREAD);
    
    // set size of character to 8
    options.c_cflag |= CS8;
    
    // disable hardware flow control
    options.c_cflag &= ~CRTSCTS;
    
    // disable canonaical mode, echo, and signal generation
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    */

    cfmakeraw(&options);
    
    
    // apply the changes
    tcsetattr(fd, TCSANOW, &options);
  }
  else{
    std::cout << "Failed to open teensy" << std::endl;
    return 1;
  }  

  while(true){
    // try reading a byte
    uint8_t byte = 0;
    int ret = read(fd, &byte, sizeof(uint8_t));

    // if a byte was read, print it
    if(ret == 1){
      std::cout << std::hex << (unsigned int)byte << std::endl;
    }

  }

  return 0;
}
