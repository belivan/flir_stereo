#include <IntervalTimer.h>
#include <TimeLib.h>

IntervalTimer timer;

const uint8_t PPS_PIN = 1;      
const uint8_t OUT_PIN1 = 2;     
const uint8_t OUT_PIN2 = 3;

volatile bool ppsReceived = false;
volatile bool state = LOW;              
volatile unsigned long ppsMicros = 0;   
unsigned long currentMicros = 0;
long drift = 0;
const unsigned long PPS_INTERVAL_US = 1000000;  
float clockFactor = 1.0;  
const unsigned long DEBOUNCE_TIME_US = 950000; 

// PID controller parameters
float Kp = 0.1;
float Ki = 0.01;
float Kd = 0.01;
float integral = 0;
float lastError = 0;

unsigned long softwareMicros = 0;
unsigned long lastSoftwareMicros = 0;
volatile time_t synchronizedTime = 0;

// We won't limit triggers now since we want continuous 10 Hz
// const int MAX_TRIGGERS_PER_SECOND = 10;
// volatile int triggerCount = 0;

const unsigned short HEADER = 0x55AA;
const unsigned short FOOTER = 0x66BB;

void togglePin() {
  // Just toggle every time (10 Hz continuously)
  digitalWrite(OUT_PIN1, state);
  digitalWrite(OUT_PIN2, state);
  state = !state;

  if (state == HIGH) {
    // Send timestamp packet every toggle or just every HIGH toggle
    currentMicros = micros();
    sendTimestampPacket(currentMicros);
  }
}

time_t getPPS() {
  noInterrupts();
  time_t currentSyncTime = synchronizedTime;
  interrupts();
  return currentSyncTime;
}

void ppsInterrupt() {
  noInterrupts();
  currentMicros = micros();
  if (currentMicros - ppsMicros > DEBOUNCE_TIME_US) {

    // No longer stop the timer. We want continuous output.
    // timer.end();
    // triggerCount = 0; // No trigger counting needed now

    unsigned long interval = currentMicros - ppsMicros;
    ppsMicros = currentMicros;

    // Calculate drift
    drift = (long)interval - (long)PPS_INTERVAL_US;

    // PID Controller for Drift Compensation
    float error = drift;
    integral += error;
    float derivative = error - lastError;
    float adjustment = (Kp * error) + (Ki * integral) + (Kd * derivative);
    lastError = error;

    // Adjust clockFactor
    clockFactor += adjustment * 1e-6;
    clockFactor = constrain(clockFactor, 0.999, 1.001);

    // Update synchronized time
    softwareMicros += (unsigned long)(PPS_INTERVAL_US * clockFactor);
    synchronizedTime = softwareMicros / 1000000;

    ppsReceived = true;

    // Reset the state to LOW on PPS if desired
    if (state == HIGH) {
      state = LOW;
      digitalWrite(OUT_PIN1, LOW);
      digitalWrite(OUT_PIN2, LOW);
    }

    // Do NOT restart the timer here. Just let it run continuously.
    // timer.begin(togglePin, 100000 / MAX_TRIGGERS_PER_SECOND); 
  }
  interrupts();
}

void setup() {
  pinMode(OUT_PIN1, OUTPUT);
  pinMode(OUT_PIN2, OUTPUT);
  pinMode(PPS_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(PPS_PIN), ppsInterrupt, RISING);

  Serial.begin(115200);

  synchronizedTime = 0;
  softwareMicros = 0;

  setSyncProvider(getPPS);
  setSyncInterval(1);

  // Start the timer once at 10Hz (100000 microseconds = 100 ms)
  timer.begin(togglePin, 100000/2);
}

void sendTimestampPacket(unsigned long timestamp) {
  // Write header
  Serial.write((uint8_t)(HEADER & 0xFF));
  Serial.write((uint8_t)(HEADER >> 8));
  
  // Write timestamp (little-endian)
  Serial.write((uint8_t)(timestamp & 0xFF));
  Serial.write((uint8_t)((timestamp >> 8) & 0xFF));
  Serial.write((uint8_t)((timestamp >> 16) & 0xFF));
  Serial.write((uint8_t)((timestamp >> 24) & 0xFF));
  
  // Write footer
  Serial.write((uint8_t)(FOOTER & 0xFF));
  Serial.write((uint8_t)(FOOTER >> 8));
}

void loop() {
  // No triggerCount logic, no stopping the timer.
  // Just continuous 10Hz toggling.
}
