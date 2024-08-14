const int LED_pin = 13;  // LED output, for testing
int output_pin1 = 6;
int output_pin2 = 7;

// GPS
int gps_trigger_pin = 8;  // GPS trigger input pin
unsigned long gps_time = 0;
unsigned long last_gps_time = 0;
volatile bool gps_triggered = false;  // flag to indicate GPS trigger

void gps_trigger() {
  gps_time = micros();
  gps_triggered = true;  // Set the flag
}

void setup() {
  pinMode(LED_pin, OUTPUT);  // pin 13 has an LED connected on most Arduino boards, set as output

  pinMode(output_pin1, OUTPUT);  // set the digital pin as output
  pinMode(output_pin2, OUTPUT);  // set the digital pin as output

  // GPS
  pinMode(gps_trigger_pin, INPUT);                                               // set the GPS trigger pin as input
  attachInterrupt(digitalPinToInterrupt(gps_trigger_pin), gps_trigger, RISING);  // attach interrupt to GPS trigger pin

  Serial.begin(115200);
}

int last_trigger_count = -1;
int trigger_count = -1;

unsigned long now = 0;
unsigned long one_sixtieth = 16666;  // 1/60th of a second in microseconds

unsigned long pulse_width = 2;  // should be 1.8us
unsigned long high_start = 0;

int gps_trigger_count = 0;

bool wait_signal_flag = true;

bool first_gps_trigger = true;

// PD Control variables
float Kp = 0.01;  // Proportional gain
float Kd = 0.001;  // Derivative gain
float last_error = 0;
float pd_correction = 0;

void loop() {
  now = micros();
  unsigned long diff = gps_time - last_gps_time;  // should be around 1 sec = 1,000,000 us

  if (gps_triggered) {//  && diff > 900000 && diff < 1100000) {

    Serial.print("Diff: ");
    Serial.println(diff);


    // Set/reset initial variables for each GPS trigger
    gps_trigger_count++;
    gps_triggered = false;
    wait_signal_flag = false;

    Serial.print("Last sensor trigger count (60): ");
    Serial.println(last_trigger_count);

    trigger_count = 0;
    last_trigger_count = 0;
    high_start = -1;

    if (!first_gps_trigger && gps_trigger_count > 15) {
      // Calculate error and derivative
      float error = (gps_time - last_gps_time) - (one_sixtieth * 60);
      float derivative = error - last_error;
      
      // PD correction
      pd_correction = (Kp * error) + (Kd * derivative);
      one_sixtieth += pd_correction;

      Serial.print("PD Correction: ");
      Serial.print(pd_correction);
      Serial.print(" Adjusted one_sixtieth to ");
      Serial.print(one_sixtieth);
      Serial.println(" us");

      last_error = error;
    }

    last_gps_time = gps_time;

    Serial.print("GPS trigger:");
    Serial.print(gps_trigger_count);
    Serial.print(" at ");
    Serial.print(gps_time);
    Serial.println(" us");

    if (gps_trigger_count % 2 == 0) {
      digitalWrite(LED_pin, HIGH);
    } else {
      digitalWrite(LED_pin, LOW);
    }

    if (first_gps_trigger) {
      first_gps_trigger = false;
    }
  }

  // Calculate trigger count based on elapsed time since last GPS pulse
  if (gps_time != 0 && !wait_signal_flag) {
    trigger_count = (now - gps_time) / one_sixtieth;
  }

  if (trigger_count != last_trigger_count) {
    // Set pin high
    high_start = now;

    digitalWrite(output_pin1, HIGH);
    //digitalWrite(output_pin2, HIGH);

    last_trigger_count = trigger_count;

    if (trigger_count % 10 == 0) {
      Serial.print("Trigger ");
      Serial.print(trigger_count);
      Serial.print(" at ");
      Serial.print(now);
      Serial.println(" us");
    }
  }

  if (now - high_start > pulse_width) {
    // Set pin low
    digitalWrite(output_pin1, LOW);
    //digitalWrite(output_pin2, LOW);
  }

  if (trigger_count > 100) {
    wait_signal_flag = true;
    Serial.println("WAIT FLAG ACTIVATED!!!!");
  }
}
