
#include <IntervalTimer.h>

IntervalTimer timer;        // Create an IntervalTimer object
const int ppsPin = 1;       // Pin where the PPS signal is connected
const int outputPin1 = 2;   // Pin to generate the signal (e.g., an LED or output pin)
const int outputPin2 = 3;   // Pin to generate the signal (e.g., an LED or output pin)
volatile bool state = LOW;  // To keep track of the pin state (HIGH or LOW)

// This function toggles the output pin
void togglePin() {
  digitalWrite(outputPin1, state);  // Set the pin to the current state (HIGH or LOW)
  digitalWrite(outputPin2, state);  // Set the pin to the current state (HIGH or LOW)
  state = !state;                  // Invert the state for the next toggle
}

// Interrupt function for PPS signal
void ppsInterrupt() {
  Serial.println("PPS Received");  // Print a message when PPS signal is received
  timer.end();                     // Stop the timer if it's running
  if (state == HIGH) { state = LOW; digitalWrite(outputPin1, state); digitalWrite(outputPin2, state);}            // If the state is HIGH, reset it to LOW
  
  timer.begin(togglePin, 8333);    // Restart the timer with a 16.67ms period (60Hz)
}

void setup() {
  pinMode(outputPin1, OUTPUT);      // Set the output pin as an output
  pinMode(outputPin2, OUTPUT);      // Set the output pin as an output
  pinMode(ppsPin, INPUT);          // Set the PPS pin as an input
  attachInterrupt(digitalPinToInterrupt(ppsPin), ppsInterrupt, RISING);  // Trigger on the rising edge of PPS signal
}

void loop() {
  // Nothing here, the work is done by the interrupt and timer
}