/*
* Copyright (c) 2016 Carnegie Mellon University, Author <shichaoy@andrew.cmu.edu>
*
* REVIEW AND EVALUATION LICENSE ONLY --- NON-COMMERCIAL RESEARCH USE ONLY
*
* Provided Sponsor has fulfilled (and continues to fulfill) any and all payment obligations to Carnegie Mellon as 
* contemplated by this Agreement, Carnegie Mellon hereby grants to Sponsor a non-exclusive, non-transferable, 
* royalty-free, perpetual license for any and all Carnegie Mellon Intellectual Property for the Sponsor's 
* internal demonstration and internal, non-commercial research use ("Review and Evaluation License").  Pursuant to 
* such Review and Evaluation License, Sponsor may copy and distribute the Deliverables to individuals internally 
* within its own organization.  Sponsor may also modify the Deliverables, provided that Sponsor may only use such 
* modifications within the scope of this Review and Evaluation License and hereby assigns to Carnegie Mellon any 
* and all rights to such modifications.  Unless source code is delivered to Sponsor, Sponsor agrees that it shall 
* not (and will not allow others to) decompile or reverse engineer any Deliverables.  Except for the rights 
* granted above, all other rights in the Deliverables remain with Carnegie Mellon.  
*
*
* If Sponsor would like additional rights to the Deliverables (including but not limited to the right to use the
* Deliverables for commercial marketing, production, redistribution, sale, rent, lease, sublicensing assignment, 
* publication, or dissemination) it must request to negotiate a commercial license as described in the agreement.
*
*/

const int LED_pin = 13;         // LED output,  for testing
int output_pin1 = 4;
int output_pin2 = 5;

uint8_t state = HIGH;
int pulse_length = 1800;

unsigned int time_since_boot_in_tenths_of_seconds = 0;
unsigned short header = 0x55aa;
unsigned short footer = 0x66bb;
char* packet;
unsigned int packet_size = sizeof(unsigned short) + sizeof(unsigned int) + sizeof(unsigned short);
volatile unsigned short pulse_count = 0;

IntervalTimer timer;

// put your setup code here, to run once:
void setup() {
  pinMode(LED_pin, OUTPUT);
  digitalWrite(LED_pin, OUTPUT);
  pinMode(output_pin1, OUTPUT);
  pinMode(output_pin2, OUTPUT);

  packet = (char*)malloc(packet_size);

  Serial.begin(9600);    // for usb serial, for testing. can directly connect to PC

  timer.begin(pulse, 100000); // 1/10th of a second
}


void pulse(){
  // turn the LED on for 1 second at the beginning of each 10 second period
  if(pulse_count == 0)
    digitalWrite(LED_pin, HIGH);
  else if(pulse_count == 10)
   digitalWrite(LED_pin, LOW);

  // build the time packet
  memcpy(&packet[0], &header, sizeof(unsigned short));
  memcpy(&packet[sizeof(unsigned short)], &time_since_boot_in_tenths_of_seconds, sizeof(unsigned int));
  memcpy(&packet[sizeof(unsigned short) + sizeof(unsigned int)], &footer, sizeof(unsigned short));
  /*
  packet[0] = reinterpret_cast<char*>(&header)[0];
  packet[1] = reinterpret_cast<char*>(&header)[1];
  packet[2] = reinterpret_cast<char*>(&time_since_boot_in_tenths_of_seconds)[0];
  packet[3] = reinterpret_cast<char*>(&time_since_boot_in_tenths_of_seconds)[1];
  packet[4] = 0x0d;//reinterpret_cast<char*>(&time_since_boot_in_tenths_of_seconds)[2];
  packet[5] = 0x0a;//reinterpret_cast<char*>(&time_since_boot_in_tenths_of_seconds)[3];
  packet[6] = reinterpret_cast<char*>(&footer)[0];
  packet[7] = reinterpret_cast<char*>(&footer)[1];
  */

  // send the packet over USB
  if(Serial)
    for(unsigned int i = 0; i < packet_size; i++)
      Serial.write(packet[i]);

  // update counters
  time_since_boot_in_tenths_of_seconds++;
  pulse_count++;
  if(pulse_count == 100)
    pulse_count = 0;
}

uint8_t opposite_state(uint8_t s){
  if(s == HIGH)
    return LOW;
  return HIGH;
}

// if delayNanoseconds tries to sleep for too long there is some rollover internally and it doesn't sleep for the right amount of time
void big_delay(int ns){
  int safe_amount = 1000000;
  int iterations = ns / safe_amount;
  int remainder = ns % safe_amount;
  for(int i = 0; i < iterations; i++)
    delayNanoseconds(safe_amount);
  if(remainder > 0)
    delayNanoseconds(remainder);
}

int period = 16666666;

void loop() {
 
  digitalWrite(output_pin1, HIGH);
  delayNanoseconds(pulse_length);
  digitalWrite(output_pin1, LOW);

  big_delay(period/2 - pulse_length);

  digitalWrite(output_pin2, HIGH);
  delayNanoseconds(pulse_length);
  digitalWrite(output_pin2, LOW);

  big_delay(period/2 - pulse_length);

 /* working with 1 pin
  digitalWrite(output_pin1, HIGH);
  delayNanoseconds(pulse_length);
  digitalWrite(output_pin1, LOW);

  uint32_t delay = 1000000;
  for(int i = 0; i < 16; i++)
    delayNanoseconds(delay);
  delayNanoseconds(666666 - pulse_length);
*/
}

