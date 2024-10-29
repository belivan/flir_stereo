#include <IntervalTimer.h>
#include <TimeLib.h>

IntervalTimer timer;

const uint8_t PPS_PIN = 1;      // Pin where the PPS signal is connected
const uint8_t OUT_PIN1 = 2;     // Pins for camera trigger outputs
const uint8_t OUT_PIN2 = 3;

volatile bool ppsReceived = false;
volatile bool state = LOW;              // To keep track of the trigger pins state (HIGH or LOW)
volatile unsigned long ppsMicros = 0;   // To keep track of the PPS signal time
long drift = 0;
const unsigned long PPS_INTERVAL_US = 1000000;  // 1 second in microseconds
float clockFactor = 1.0;  // Clock factor to adjust the drift
const unsigned long DEBOUNCE_TIME_US = 950000; // 100 microseconds debounce time

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

// Variables to limit triggers to 10 per second
const int MAX_TRIGGERS_PER_SECOND = 30;
volatile int triggerCount = 0;

// Function to toggle the output pins
void togglePin() {
  // Check if we are within the 10 triggers limit
  if (triggerCount < MAX_TRIGGERS_PER_SECOND) {
    // Toggle the output pins
    digitalWrite(OUT_PIN1, state);
    digitalWrite(OUT_PIN2, state);
    state = !state;

    // Increment the trigger count
    triggerCount++;
  } else {
    // Keep the state LOW after 10 triggers
    if (state == HIGH) {
      Serial.println("Reached 10Hz");
      state = LOW;
      digitalWrite(OUT_PIN1, LOW);
      digitalWrite(OUT_PIN2, LOW);
    }
  }
}

time_t getPPS() {
  noInterrupts();
  time_t currentSyncTime = synchronizedTime;
  interrupts();
  return currentSyncTime;
}

// Interrupt function for PPS signal
void ppsInterrupt() {
  noInterrupts();
  unsigned long currentMicros = micros();
  if (currentMicros - ppsMicros > DEBOUNCE_TIME_US) {
    // Stop the timer
    timer.end();

    // Reset the trigger count for the new second
    triggerCount = 0;

    // Calculate interval since last PPS
    unsigned long interval = currentMicros - ppsMicros;
    ppsMicros = currentMicros;

    // Calculate drift (difference from expected interval)
    drift = (long)interval - (long)PPS_INTERVAL_US;

    // PID Controller for Drift Compensation
    float error = drift;                  // Current error
    integral += error;                    // Accumulate integral
    float derivative = error - lastError; // Calculate derivative
    float adjustment = (Kp * error) + (Ki * integral) + (Kd * derivative);
    lastError = error;

    // Adjust clockFactor based on PID output
    clockFactor += adjustment * 1e-6; // Microseconds to small factor

    // Clamp clockFactor to prevent excessive adjustments
    clockFactor = constrain(clockFactor, 0.999, 1.001);

    // Update the synchronized time
    softwareMicros += (unsigned long)(PPS_INTERVAL_US * clockFactor);
    synchronizedTime = softwareMicros / 1000000; // Convert to seconds

    ppsReceived = true;

    // Reset the state to LOW on PPS
    if (state == HIGH) {
      state = LOW;
    }

    // Restart the timer to trigger the cameras, limiting to 10 triggers per second
    timer.begin(togglePin, 100000 / MAX_TRIGGERS_PER_SECOND); // 100ms / 10 triggers = 10Hz
  }
  interrupts();
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
    Serial.print(clockFactor, 9);
    Serial.print(" | Drift: ");
    Serial.println(drift);
  }
  
  static time_t lastPrint = 0;
  time_t currentTime = now();
  
  if (currentTime != lastPrint) {
    Serial.print("Time Now: ");
    Serial.print(hour());
    Serial.print(":");
    Serial.print(minute());
    Serial.print(":");
    Serial.println(second());
    lastPrint = currentTime;

    // print hz
    Serial.print("Hz: ");
    Serial.println(triggerCount);


  }
}
