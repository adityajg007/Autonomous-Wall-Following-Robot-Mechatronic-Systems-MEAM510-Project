#define LEDC_CHANNEL_LEFT 1 // use the first channel for the left motor.
#define LEDC_CHANNEL_RIGHT 0 // use the second channel for the right motor.
#define LEDC_RESOLUTION_BITS 14
#define LEDC_RESOLUTION  ((1 << LEDC_RESOLUTION_BITS) - 1)
#define LEDC_FREQ_HZ 1000 // greater than 39000/2^14 = 2.3 Hz

// Motor Right
#define RIGHTMOTOR_PIN 18 // PWM pin for motor.
#define RIGHTMOTORD1_PIN 19 // Direction pin 1 for motor.
#define RIGHTMOTORD2_PIN 21 // Direction pin 2 for motor.

// Motor Left
#define LEFTMOTOR_PIN 17 // PWM pin for motor.
#define LEFTMOTORD1_PIN 16 // Direction pin 1 for motor.
#define LEFTMOTORD2_PIN 14 // Direction pin 2 for motor.

#define DUTY 100 // Duty cycle in % for PWM.

void setup() {
  // Left
  pinMode(LEFTMOTOR_PIN, OUTPUT);
  pinMode(LEFTMOTORD1_PIN, OUTPUT);
  pinMode(LEFTMOTORD2_PIN, OUTPUT);

  ledcSetup(LEDC_CHANNEL_LEFT, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttachPin(LEFTMOTOR_PIN, LEDC_CHANNEL_LEFT);

  //Right
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
  
  ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.5); // 0.5 to reduce speed for both
  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.76 * 0.5); // 0.76 correction to go straight

  digitalWrite(LEFTMOTORD1_PIN, LOW);
  digitalWrite(LEFTMOTORD2_PIN, HIGH);

  digitalWrite(RIGHTMOTORD1_PIN, LOW);
  digitalWrite(RIGHTMOTORD2_PIN, HIGH);
}

void drive_backward() {
  
  ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.5); // 0.5 to reduce speed for both
  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.76 * 0.5); // 0.76 correction to go straight

  digitalWrite(LEFTMOTORD1_PIN, HIGH);
  digitalWrite(LEFTMOTORD2_PIN, LOW);

  digitalWrite(RIGHTMOTORD1_PIN, HIGH);
  digitalWrite(RIGHTMOTORD2_PIN, LOW);
}

void drive_left() {

  ledcAnalogWrite(LEDC_CHANNEL_LEFT, 0);
  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.76 * 0.5); // 0.76 correction to go straight

  digitalWrite(LEFTMOTORD1_PIN, HIGH);
  digitalWrite(LEFTMOTORD2_PIN, LOW);

  digitalWrite(RIGHTMOTORD1_PIN, LOW);
  digitalWrite(RIGHTMOTORD2_PIN, HIGH);

}

void drive_right() {

  ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.5); // 0.5 to reduce speed
  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, 0);

  digitalWrite(LEFTMOTORD1_PIN, LOW);
  digitalWrite(LEFTMOTORD2_PIN, HIGH);

  digitalWrite(RIGHTMOTORD1_PIN, LOW);
  digitalWrite(RIGHTMOTORD2_PIN, HIGH);

}

void loop() {
drive_forward();
}