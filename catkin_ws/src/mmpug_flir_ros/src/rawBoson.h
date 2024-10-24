// version 0.1.1 : adding ascii mode. rawBoson a ....
// version 0.1.2 : fixing the send command. using buffer instead of byte by byte
// version 0.1.3 : Fixing the HELP menu to include working examples.
// version 0.1.4 : Translating to english
// version 1.0.0 : Fix CRC issue, start using GitHub tags to track versions.
// version 1.0.1 : Fix serial.c (setup_serial) bug. Credits to serial library authors 

#ifndef RAWBOSON_H
#define RAWBOSON_H

#include "serial.h"
#include "bytes.h"
#include <string>
#include <unistd.h>
#include <time.h>

// Define COLOR CODES
#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define BLU   "\x1B[34m"
#define MAG   "\x1B[35m"
#define CYN   "\x1B[36m"
#define WHT   "\x1B[37m"
#define RESET "\x1B[0m"

#define SetFontYellow()    printf(YEL);   /* Yellow */
#define SetFontWhite()     printf(WHT);   /* White */
#define SetFontRed()       printf(RED);   /* Red */
#define SetFontGreen()     printf(GRN);   /* Green */
#define SetFontBlue()      printf(BLU);   /* Blue */
#define SetFontCyan()      printf(CYN);   /* Cyan  */
#define SetFontMagenta()   printf(MAG);   /* Magenta  */
#define SetFontReset()     printf(RESET); /* Reset */

// Define error codes

#define NoError                           0x00

#define R_CAM_DSPCH_BAD_CMD_ID            0x0161
#define R_CAM_DSPCH_BAD_PAYLOAD_STATUS    0x0162
#define R_CAM_PKG_UNSPECIFIED_FAILURE     0x0170
#define R_CAM_PKG_INSUFFICIENT_BYTES      0x017D
#define R_CAM_PKG_EXCESS_BYTES            0x017E
#define R_CAM_PKG_BUFFER_OVERFLOW         0x017F
#define FLIR_RANGE_ERROR                  0x0203  

void print_buffer(unsigned char *buffer, int bufferlen);

void shutter(const std::string serial_port);
void set_gain_mode(int gain_mode, const std::string serial_port);
int get_gain_mode(const std::string serial_port);
void set_ffc_mode(int ffc_mode, const std::string serial_port);
int get_ffc_mode(const std::string serial_port);
int get_ffc_status(const std::string serial_port);
void set_sync_mode(int sync_mode, const std::string serial_port);
int get_sync_mode(const std::string serial_port);

#endif
