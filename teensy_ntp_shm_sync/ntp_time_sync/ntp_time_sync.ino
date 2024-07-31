const int LED_pin = 13;  // LED output,  for testing
int output_pin1 = 6;
int output_pin2 = 7;

// GPS
int gps_trigger_pin = 8;  // GPS trigger input pin
unsigned long gps_time = 0;
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

void loop() {
  now = micros();
  if (gps_triggered) {
    gps_trigger_count++;
    gps_triggered = false;
    wait_signal_flag = false;

    trigger_count = -1;
    last_trigger_count = -1;
    high_start = -1;

    Serial.print("GPS trigger at ");
    Serial.print(gps_time);
    Serial.println(" us");

    if (gps_trigger_count % 2 == 0) {
      digitalWrite(LED_pin, HIGH);
    } else {
      digitalWrite(LED_pin, LOW);
    }
  }
  if (gps_time != 0 && !wait_signal_flag) {
    trigger_count = (now - gps_time) / one_sixtieth;
  }

  if (trigger_count != last_trigger_count) {
    // set pin high
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

    if (trigger_count > 60) {
      Serial.println("CROSSED 60 TRIGGERS!");
    }
  }
  if (now - high_start > pulse_width) {
    // set pin low
    digitalWrite(output_pin1, LOW);
    //digitalWrite(output_pin2, LOW);
  }

  if (trigger_count > 100) {
    wait_signal_flag = true;
  }
}
