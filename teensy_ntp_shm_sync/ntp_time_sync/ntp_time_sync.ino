#include <IntervalTimer.h>
#include <TinyGPS++.h>
#include <TimeLib.h>

// Create objects for the timer and GPS
IntervalTimer timer;
TinyGPSPlus gps;

const int ppsPin = 1;       // Pin where the PPS signal is connected
const int gpsRxPin = 9;     // GPS module TX pin connected to Teensy's RX pin
const int outputPin1 = 2;   // Pin to generate the signal (e.g., an LED or output pin)
const int outputPin2 = 3;   // Pin to generate the signal (e.g., an LED or output pin)

volatile bool ppsReceived = false;  // Flag to indicate PPS received
volatile bool state = LOW;          // To keep track of the pin state (HIGH or LOW)

// Function to toggle the output pins
void togglePin() {
  digitalWrite(outputPin1, state);  // Set outputPin1 to the current state
  digitalWrite(outputPin2, state);  // Set outputPin2 to the current state
  state = !state;                   // Toggle the state for the next invocation
}

// Interrupt function for PPS signal
void ppsInterrupt() {
  ppsReceived = true;  // Set flag when PPS is received
  timer.end();         // Stop the timer if it's running

  // Reset the state to LOW on PPS
  if (state == HIGH) {
    state = LOW;
    digitalWrite(outputPin1, state);
    digitalWrite(outputPin2, state);
  }

  // Restart the timer with a 16.67ms period (60Hz)
  timer.begin(togglePin, 8333 * 6);
}

// Function to update system time from GPS
void updateTimeFromGPS() {
  if (gps.time.isValid() && gps.date.isValid()) {
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(),
            gps.date.day(), gps.date.month(), gps.date.year());
  }
}

// Setup function
void setup() {
  // Initialize pins
  pinMode(outputPin1, OUTPUT);
  pinMode(outputPin2, OUTPUT);
  pinMode(ppsPin, INPUT);

  // Attach the PPS interrupt
  attachInterrupt(digitalPinToInterrupt(ppsPin), ppsInterrupt, RISING);

  // Initialize serial communication with GPS
  Serial.begin(9600);  // GPS module typically communicates at 9600 baud
  Serial1.begin(9600, SERIAL_8N1, gpsRxPin, -1);

  // Initialize the system time synchronization with PPS
  setSyncProvider(ppsSync);
  setSyncInterval(1);  // Sync every second
}

// Sync function that gets called at each PPS
time_t ppsSync() {
  if (ppsReceived) {
    ppsReceived = false;
    updateTimeFromGPS();  // Update the time based on GPS data
  }
  return now();  // Resynchronize system time with the PPS signal
}

// Main loop
void loop() {
  // Continuously read GPS data
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // Debugging: Print the current time
  if (timeStatus() == timeSet) {
    Serial.print("Current Time: ");
    Serial.print(hour());
    Serial.print(":");
    Serial.print(minute());
    Serial.print(":");
    Serial.println(second());
  } else {
    Serial.println("Time not set");
  }

  delay(1000);  // Delay for readability of the output
}

void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (Serial2.available()) {
      gps.encode(Serial2.read());  // Feed GPS data to the TinyGPSPlus object
    }
  } while (millis() - start < ms);
}