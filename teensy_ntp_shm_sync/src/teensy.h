#ifndef _TEENSY_H_
#define _TEENSY_H_

#include <stdint.h>
#include <string>

struct TeensyPacket {
  unsigned int time_since_boot_in_tenths_of_seconds;
};

class Teensy {
private:
  // teensy file descriptor
  int fd;

  // teensy packet header
  static const uint8_t HEADER_FIRST_BYTE = 0xaa;
  static const uint8_t HEADER_SECOND_BYTE = 0x55;
  static const uint8_t FOOTER_FIRST_BYTE = 0xbb;
  static const uint8_t FOOTER_SECOND_BYTE = 0x66;
  
  // states
  static const int HEADER1 = 0;
  static const int HEADER2 = 1;
  static const int PAYLOAD = 2;
  static const int FOOTER1 = 3;
  static const int FOOTER2 = 4;
  int state;
  
  static const int PAYLOAD_SIZE = sizeof(unsigned int);
  int payload_count;

  TeensyPacket packet;
  
public:
  Teensy(std::string device);
  bool get_packet(TeensyPacket* packet);
};

#endif
