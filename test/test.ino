/------------------------------------------------------------------------/
/*                             5. Motor Driver                            */
/------------------------------------------------------------------------/

#define LEDC_CHANNEL_LEFT 1 // use the first channel for the left motor.
#define LEDC_CHANNEL_RIGHT 0 // use the second channel for the right motor.
#define LEDC_RESOLUTION_BITS 14
#define LEDC_RESOLUTION  ((1 << LEDC_RESOLUTION_BITS) - 1)
#define LEDC_FREQ_HZ 500 // greater than 39000/2^14 = 2.3 Hz

#define LEFTMOTOR_PIN 26 // PWM pin for motor.
#define LEFTMOTORD1_PIN 27 // Direction pin 1 for motor.
#define LEFTMOTORD2_PIN 14 // Direction pin 2 for motor.

#define RIGHTMOTOR_PIN 13 // PWM pin for motor.
#define RIGHTMOTORD1_PIN 9 // Direction pin 1 for motor.
#define RIGHTMOTORD2_PIN 10 // Direction pin 2 for motor.
#define DUTY 50 // Duty cycle for PWM.

void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(LEFTMOTOR_PIN, OUTPUT);
  pinMode(LEFTMOTORD1_PIN, OUTPUT);
  pinMode(LEFTMOTORD2_PIN, OUTPUT);
  ledcSetup(LEDC_CHANNEL_LEFT, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttachPin(LEFTMOTOR_PIN, LEDC_CHANNEL_LEFT);

  pinMode(RIGHTMOTOR_PIN, OUTPUT);
  pinMode(RIGHTMOTORD1_PIN, OUTPUT);
  pinMode(RIGHTMOTORD2_PIN, OUTPUT);
  ledcSetup(LEDC_CHANNEL_RIGHT, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttachPin(RIGHTMOTOR_PIN, LEDC_CHANNEL_RIGHT);
}

// can set syntax to be like analogWrite() with input[ 0 : valueMax ]:
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 100) {
  uint32_t duty = LEDC_RESOLUTION * min(value, valueMax) / valueMax;
  ledcWrite(channel, duty);
}

void drive_forward() {
  ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY);
  digitalWrite(LEFTMOTORD1_PIN, HIGH);
  digitalWrite(LEFTMOTORD2_PIN, LOW);

  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY);
  digitalWrite(RIGHTMOTORD1_PIN, HIGH);
  digitalWrite(RIGHTMOTORD2_PIN, LOW);
}

void drive_backward() {
  ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY);
  digitalWrite(LEFTMOTORD1_PIN, LOW);
  digitalWrite(LEFTMOTORD2_PIN, HIGH);

  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY);
  digitalWrite(RIGHTMOTORD1_PIN, LOW);
  digitalWrite(RIGHTMOTORD2_PIN, HIGH);
}

void drive_left() {
  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY);
  digitalWrite(RIGHTMOTORD1_PIN, HIGH);
  digitalWrite(RIGHTMOTORD2_PIN, LOW);
}

void drive_right() {
  ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY);
  digitalWrite(LEFTMOTORD1_PIN, HIGH);
  digitalWrite(LEFTMOTORD2_PIN, LOW);
}

void loop() {
  drive_forward();
}
