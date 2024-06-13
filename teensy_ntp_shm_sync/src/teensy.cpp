#include "teensy.h"
#include <iostream>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

Teensy::Teensy(std::string device){
  fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  
  // If the device has been opened successfully, configure it
  if(fd != -1) {
    // set non blocking
    fcntl(fd, F_SETFL, FNDELAY);
    
    struct termios options;
    tcgetattr(fd, &options);

    // set baud rate
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    // set raw input mode
    cfmakeraw(&options);
    
    // apply the changes
    tcsetattr(fd, TCSANOW, &options);
  }
  else{
    std::cout << "Failed to open teensy at " << device << std::endl;
    exit(1);
  }  
  
  state = HEADER1;
  payload_count = 0;
}

bool Teensy::get_packet(TeensyPacket* packet){
  uint8_t byte;
  int ret = read(fd, &byte, sizeof(uint8_t));

  if(!(ret == -1 || ret == 1)){
    std::cout << "BAD READ RETURN VAL: " << ret << std::endl;
    std::cout << std::strerror(errno) << std::endl;
  }
  
  if(ret != -1){
    //std::cout << "state: " << state << " byte: " << std::hex << (unsigned int)byte << std::endl;
    
    switch(state){
    case HEADER1:
      memset(&(this->packet), 0, sizeof(TeensyPacket));
      if(byte == HEADER_FIRST_BYTE)
	state = HEADER2;
      break;
    case HEADER2:
      if(byte == HEADER_SECOND_BYTE)
	state = PAYLOAD;
      else
	state = HEADER1;
      break;
    case PAYLOAD:
      if(payload_count < PAYLOAD_SIZE){
	this->packet.time_since_boot_in_tenths_of_seconds |= byte << 8*payload_count;
      }
      payload_count++;
      
      if(payload_count >= PAYLOAD_SIZE){
	payload_count = 0;
	state = FOOTER1;
      }
      break;
    case FOOTER1:
      if(byte == FOOTER_FIRST_BYTE)
	state = FOOTER2;
      else
	state = HEADER1;
      break;
    case FOOTER2:
      state = HEADER1;
      if(byte == FOOTER_SECOND_BYTE){
	*packet = this->packet;
	return true;
      }
      break;
    }
  }
  
  return false;
}
