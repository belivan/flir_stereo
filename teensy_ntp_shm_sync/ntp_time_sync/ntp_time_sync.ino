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

// const int LED_pin = 13;  // LED output, for testing, not being used
// int output_pin1 = 2;
// int output_pin2 = 3;


// // GPS
// int gps_trigger_pin = 1;  // GPS trigger input pin
// unsigned long gps_time = 0;
// unsigned long last_gps_time = 0;
// volatile bool gps_triggered = false;  // flag to indicate GPS trigger

// // Package parameters
// unsigned int time_since_boot_in_tenths_of_seconds = 0;  
// unsigned short header = 0x55aa;
// unsigned short footer = 0x66bb;
// char* packet;
// unsigned int packet_size = sizeof(unsigned short) + sizeof(unsigned int) + sizeof(unsigned short);  // 8 bytes total? 2 + 4 + 2
// volatile unsigned short pulse_count = 0;
// IntervalTimer timer;

// void gps_trigger() {
//   gps_time = micros();
//   gps_triggered = true;  // Set the flag
// }

// void setup() {
//   pinMode(LED_pin, OUTPUT);  // pin 13 has an LED connected on most Arduino boards, set as output

//   pinMode(output_pin1, OUTPUT);  // set the digital pin as output
//   pinMode(output_pin2, OUTPUT);  // set the digital pin as output

//   // Packet
//   packet = (char*)malloc(packet_size);  // allocate memory for the packet
  
//   // GPS
//   pinMode(gps_trigger_pin, INPUT);                                               // set the GPS trigger pin as input
//   attachInterrupt(digitalPinToInterrupt(gps_trigger_pin), gps_trigger, RISING);  // attach interrupt to GPS trigger pin

//   Serial.begin(115200);

//   // Timer
//   Serial.println("Starting timer at 10Hz!!!");
//   timer.begin(pulse, 100000);  // 100000 ns = 0.1ms = 10Hz
// }

// void pulse(){  // this function is called every 0.1ms = 10Hz ???
//   // turn the LED on for 1 second at the beginning of each 10 second period
//   // if(pulse_count == 0)
//   //   digitalWrite(LED_pin, HIGH);
//   // else if(pulse_count == 10)
//   //  digitalWrite(LED_pin, LOW);

//   // build the time packet
//   memcpy(&packet[0], &header, sizeof(unsigned short));  // copy the header into the packet
//   memcpy(&packet[sizeof(unsigned short)], &time_since_boot_in_tenths_of_seconds, sizeof(unsigned int));  // copy the time into the packet
//   memcpy(&packet[sizeof(unsigned short) + sizeof(unsigned int)], &footer, sizeof(unsigned short));  // copy the footer into the packet
//   /*
//   packet[0] = reinterpret_cast<char*>(&header)[0];
//   packet[1] = reinterpret_cast<char*>(&header)[1];
//   packet[2] = reinterpret_cast<char*>(&time_since_boot_in_tenths_of_seconds)[0];
//   packet[3] = reinterpret_cast<char*>(&time_since_boot_in_tenths_of_seconds)[1];
//   packet[4] = 0x0d;//reinterpret_cast<char*>(&time_since_boot_in_tenths_of_seconds)[2];
//   packet[5] = 0x0a;//reinterpret_cast<char*>(&time_since_boot_in_tenths_of_seconds)[3];
//   packet[6] = reinterpret_cast<char*>(&footer)[0];
//   packet[7] = reinterpret_cast<char*>(&footer)[1];
//   */

//   // send the packet over USB
//   if(Serial)  // if the serial port is open
//     for(unsigned int i = 0; i < packet_size; i++)
//       Serial.write(packet[i]);

//   // update counters
//   time_since_boot_in_tenths_of_seconds++;
//   // pulse_count++;
//   // if(pulse_count == 100)
//   //   pulse_count = 0;
// }

// int last_trigger_count = -1;
// int trigger_count = -1;

// unsigned long now = 0;
// unsigned long one_sixtieth = 16666;  // 1/60th of a second in microseconds

// unsigned long pulse_width = 1;  // should be 1.8us
// unsigned long high_start = 0;

// int gps_trigger_count = 0;

// bool wait_signal_flag = true;

// bool first_gps_trigger = true;

// // PD Control variables, really not needed, overkill
// // float Kp = -0.01;  // Proportional gain
// // float Kd = -0.001;  // Derivative gain
// // float last_error = 0;
// // float pd_correction = 0;

// void loop() {
//   now = micros();
//   unsigned long diff = gps_time - last_gps_time;  // should be around 1 sec = 1,000,000 us

//   if (gps_triggered) {//  && diff > 900000 && diff < 1100000) {

//     Serial.print("Diff: ");
//     Serial.println(diff);


//     // Set/reset initial variables for each GPS trigger
//     gps_trigger_count++;
//     gps_triggered = false;
//     wait_signal_flag = false;

//     Serial.print("Last sensor trigger count (60): ");
//     Serial.println(last_trigger_count);

//     trigger_count = 0;
//     last_trigger_count = 0;
//     high_start = -1;

//     // if (!first_gps_trigger && gps_trigger_count > 15) {
//     //   // Calculate error and derivative
//     //   float error = (gps_time - last_gps_time) - (one_sixtieth * 60);
//     //   float derivative = error - last_error;
      
//     //   // PD correction
//     //   pd_correction = (Kp * error) + (Kd * derivative);
//     //   one_sixtieth += pd_correction;

//     //   Serial.print("PD Correction: ");
//     //   Serial.print(pd_correction);
//     //   Serial.print(" Adjusted one_sixtieth to ");
//     //   Serial.print(one_sixtieth);
//     //   Serial.println(" us");

//     //   last_error = error;
//     // }

//     last_gps_time = gps_time;

//     Serial.print("GPS trigger:");
//     Serial.print(gps_trigger_count);
//     Serial.print(" at ");
//     Serial.print(gps_time);
//     Serial.println(" us");

//     if (gps_trigger_count % 2 == 0) {
//       digitalWrite(LED_pin, HIGH);
//     } else {
//       digitalWrite(LED_pin, LOW);
//     }

//     // if (first_gps_trigger) {
//     //   first_gps_trigger = false;
//     // }
//   }

//   // Calculate trigger count based on elapsed time since last GPS pulse
//   if (gps_time != 0 && !wait_signal_flag) {
//     trigger_count = (now - gps_time) / one_sixtieth;
//   }

//   if (trigger_count != last_trigger_count) {
//     // Set pin high
//     high_start = now;

//     digitalWrite(output_pin1, HIGH);
//     digitalWrite(output_pin2, HIGH);

//     last_trigger_count = trigger_count;

//     if (trigger_count % 10 == 0) {
//       Serial.print("Trigger ");
//       Serial.print(trigger_count);
//       Serial.print(" at ");
//       Serial.print(now);
//       Serial.println(" us");
//     }
//   }

//   if (now - high_start > pulse_width) {
//     // Set pin low
//     digitalWrite(output_pin1, LOW);
//     digitalWrite(output_pin2, LOW);
//   }

//   if (trigger_count > 100) {
//     wait_signal_flag = true;
//     Serial.println("WAIT FLAG ACTIVATED!!!!");
//   }
// }



const int LED_pin = 13;         // LED output,  for testing
int output_pin1 = 2;
int output_pin2 = 3;

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
  int safe_amount = 100000;
  int iterations = ns / safe_amount;
  int remainder = ns % safe_amount;
  for(int i = 0; i < iterations; i++)
    delayNanoseconds(safe_amount);
  if(remainder > 0)
    delayNanoseconds(remainder);
}

int period = 16666666;

void loop() {
  // NEW SIGNAL 60HZ and 50% duty cycle
  digitalWrite(output_pin1, HIGH);
  big_delay(period/2)
  digitalWrite(output_pin1, LOW);
  big_delay(period/2)
 
  // digitalWrite(output_pin1, HIGH);
  // delayNanoseconds(pulse_length);
  // digitalWrite(output_pin1, LOW);

  // big_delay(period/2 - pulse_length);

  // digitalWrite(output_pin2, HIGH);
  // delayNanoseconds(pulse_length);
  // digitalWrite(output_pin2, LOW);

  //delay(16);
  //big_delay(period/2 - pulse_length);

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










