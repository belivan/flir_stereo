#include <IntervalTimer.h>
#include <TimeLib.h>

IntervalTimer timer;

const uint8_t PPS_PIN = 1;      // Pin where the PPS signal is connected
const uint8_t OUT_PIN1 = 2;  // Pins for camera trigger outputs
const uint8_t OUT_PIN2 = 3;

volatile bool ppsReceived = false;
volatile bool state = LOW;  // To keep track of the trigger pins state (HIGH or LOW)
volatile unsigned long ppsMicros = 0;  // To keep track of the PPS signal time
long drift = 0;
const unsigned long PPS_INTERVAL_US = 1000000;  // 1 second in microseconds
float clockFactor = 1.0; // Clock factor to adjust the drift
const unsigned long DEBOUNCE_TIME_US = 100; // 100 microseconds debounce time

// PID controller parameters
float Kp = 0.1;
float Ki = 0.01;
float Kd = 0.01;
float integral = 0;
float lastError = 0;

// Software clock variables
unsigned long softwareMicros = 0;
unsigned long lastSoftwareMicros = 0;
volatile time_t synchronizedTime = 0;

// Function to toggle the output pins
void togglePin() {
// The idea is to toggle the output pins every half period,
// i.e., if the period is 16.67ms (60Hz), the output pins will be toggled every 8.33ms
// if 10 Hz is desired, the period will be 100ms, and the output pins will be toggled every 50ms
  digitalWrite(OUT_PIN1, state);
  digitalWrite(OUT_PIN2, state);
  state = !state;
}

time_t getPPS() {
  noInterrupts();
  time_t currentSyncTime = synchronizedTime;
  interrupts();
  return currentSyncTime;
}

// Interrupt function for PPS signal
void ppsInterrupt() {
  unsigned long currentMicros = micros();
  if (currentMicros - ppsMicros > DEBOUNCE_TIME_US) {
    // Stop the timer
    timer.end();

    // Calculate interval since last PPS
    unsigned long interval = currentMicros - ppsMicros;
    ppsMicros = currentMicros;

    // Calculate drift (difference from expected interval)
    long drift = (long)interval - (long)PPS_INTERVAL_US;

    // PID Controller for Drift Compensation
    float error = drift;                  // Current error
    integral += error;                    // Accumulate integral
    float derivative = error - lastError; // Calculate derivative
    float adjustment = (Kp * error) + (Ki * integral) + (Kd * derivative);
    lastError = error;

    // Adjust clockFactor based on PID output
    clockFactor += adjustment * 1e-6; // Microseconds to small factor

    // Clamp clockFactor to prevent excessive adjustments, see if needed
    clockFactor = constrain(clockFactor, 0.999, 1.001);

    // Update the synchronized time
    // Increment by one second, adjusted by clockFactor
    softwareMicros += (unsigned long)(PPS_INTERVAL_US * clockFactor);
    synchronizedTime = softwareMicros / 1000000; // Convert to seconds

    ppsReceived = true;

    // Reset the state to LOW on PPS, check if this causes any issues!!!
    if (state == HIGH) {
      state = LOW;
    }

    // Restart the timer
    timer.begin(togglePin, 8333); // Currently set to 4Hz
  }
}

// Setup function
void setup() {
  // Initialize pins
  pinMode(OUT_PIN1, OUTPUT);
  pinMode(OUT_PIN2, OUTPUT);
  pinMode(PPS_PIN, INPUT);

  // Attach the PPS interrupt
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), ppsInterrupt, RISING);

  // Initialize the system time synchronization with PPS
  // setSyncProvider(ppsSync);
  // setSyncInterval(1);  // Sync every second

  Serial.begin(115200);

  synchronizedTime = 0;
  softwareMicros = 0;

  setSyncProvider(getPPS);
  setSyncInterval(1);

  // Check if TimeLib synchronization is successful
  if (timeStatus() != timeSet) {
    Serial.println("Time not set!");
  } else {
    Serial.println("Time set successfully.");
  }
}

// Main loop
void loop() {
  if(ppsReceived) {
    ppsReceived = false;
    Serial.print("PPS Received ");
    Serial.print("| Synchronized Time: ");
    Serial.print(synchronizedTime);
    Serial.print(" | Clock Factor: ");
    Serial.println(clockFactor, 9);
  }
  
  static time_t lastPrint = 0;
  time_t currentTime = now();
  
  if (currentTime != lastPrint) {
    Serial.print("Current Time: ");
    Serial.print(hour());
    Serial.print(":");
    Serial.print(minute());
    Serial.print(":");
    Serial.println(second());
    lastPrint = currentTime;
  }
}

// Not used
// void smartDelay(unsigned long ms) {
//   unsigned long start = millis();
//   do {
//     while (Serial2.available()) {
//       gps.encode(Serial2.read());  // Feed GPS data to the TinyGPSPlus object
//     }
//   } while (millis() - start < ms);
// }