//********************** Definitions ***********************

// #include <WiFi.h>
// #include <WiFiUdp.h>
// #include "html510.h" // HTML library for UDP communication.

// -------------------- Define Motor Pin --------------------------
#define LEDC_CHANNEL_LEFT 1 // use the first channel for the left motor.
#define LEDC_CHANNEL_RIGHT 0 // use the second channel for the right motor.
#define LEDC_RESOLUTION_BITS 14
#define LEDC_RESOLUTION  ((1 << LEDC_RESOLUTION_BITS) - 1)
#define LEDC_FREQ_HZ 1000 // greater than 39000/2^14 = 2.3 Hz

// Motor Right
#define RIGHTMOTOR_PIN 33 // PWM pin for motor.
#define RIGHTMOTORD1_PIN 19 // Direction pin 1 for motor.
#define RIGHTMOTORD2_PIN 21 // Direction pin 2 for motor.

// Motor Left
#define LEFTMOTOR_PIN 17 // PWM pin for motor.
#define LEFTMOTORD1_PIN 16 // Direction pin 1 for motor.
#define LEFTMOTORD2_PIN 14 // Direction pin 2 for motor.

#define DUTY 100 // Duty cycle in % for PWM.

//--------------------- Define IR Pin -------------------------
// Define the pins where the sensors are connected
#define SENSOR_PIN_1 35 // Front IR
#define SENSOR_PIN_2 10 // Right IR
#define SENSOR_PIN_3 36 // Left IR

// Structure to hold the status of all sensors
struct SensorStatus {
  bool front;
  bool right;
  bool left;
};

//********************* Setup ************************
void setup() {
  //--------------------- IR Setup -------------------------
  Serial.begin(115200); // Start the serial communication at 115200 baud rate

  // Set the sensor pins as input
  pinMode(SENSOR_PIN_1, INPUT);
  pinMode(SENSOR_PIN_2, INPUT);
  pinMode(SENSOR_PIN_3, INPUT);

  //--------------------- Motor Setup -------------------------
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

//******************** Motor Driving Functions *************************
//--------------------- Motor Driving Functions -------------------------
// can set syntax to be like analogWrite() with input[ 0 : valueMax ]:
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 100) {
  uint32_t duty = LEDC_RESOLUTION * min(value, valueMax) / valueMax;
  ledcWrite(channel, duty);
}

void drive_forward() {
  
  ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.5);
  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.5);

  digitalWrite(LEFTMOTORD1_PIN, LOW);
  digitalWrite(LEFTMOTORD2_PIN, HIGH);

  digitalWrite(RIGHTMOTORD1_PIN, LOW);
  digitalWrite(RIGHTMOTORD2_PIN, HIGH);
}

void drive_backward() {
  
  ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.5); // 0.5 to reduce speed for both
  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.5); // 0.78 correction to go straight

  digitalWrite(LEFTMOTORD1_PIN, HIGH);
  digitalWrite(LEFTMOTORD2_PIN, LOW);

  digitalWrite(RIGHTMOTORD1_PIN, HIGH);
  digitalWrite(RIGHTMOTORD2_PIN, LOW);
}

void drive_left() {

  ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.5);
  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.5); // 0.76 correction to go straight

  digitalWrite(LEFTMOTORD1_PIN, HIGH);
  digitalWrite(LEFTMOTORD2_PIN, LOW);

  digitalWrite(RIGHTMOTORD1_PIN, LOW);
  digitalWrite(RIGHTMOTORD2_PIN, HIGH);

}

void drive_right() {

  ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.5); // 0.5 to reduce speed
  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.5);

  digitalWrite(LEFTMOTORD1_PIN, LOW);
  digitalWrite(LEFTMOTORD2_PIN, HIGH);

  digitalWrite(RIGHTMOTORD1_PIN, HIGH);
  digitalWrite(RIGHTMOTORD2_PIN, LOW);

}

unsigned long rightBlockedStartTime = 0; // Variable to store the time when right sensor first gets blocked

//******************** Loop *************************
void loop() {
  //--------------------- IR Loop -------------------------
  // Read the sensor values
  SensorStatus sensor_status;
  sensor_status.front = digitalRead(SENSOR_PIN_1) == HIGH;
  sensor_status.right = digitalRead(SENSOR_PIN_2) == HIGH;
  sensor_status.left = digitalRead(SENSOR_PIN_3) == HIGH;

  while (true) {
      if (digitalRead(SENSOR_PIN_1) == HIGH && digitalRead(SENSOR_PIN_2) == HIGH && digitalRead(SENSOR_PIN_3) == HIGH) {
        // If eveything is not blocked
        delay(200); // Delay for 200 ms
        drive_left(); // Continue straight forward
        // break; // Exit the while loop
      } else if (digitalRead(SENSOR_PIN_1) == LOW) {
        // If front is still blocked
        drive_forward(); // Turn to the left for 200 ms
        delay(200); // Delay for 200 ms
      }
    }

  // Use the sensor_status variable in an if-else conditional
  /*if (sensor_status.front && sensor_status.right && sensor_status.left) {
    Serial.println("Nothing detected");
    }
    if (!sensor_status.front && sensor_status.right && sensor_status.left) {
    Serial.println("Front blocked");
    }
    if (sensor_status.front && !sensor_status.right && sensor_status.left) {
    Serial.println("Right blocked");
    }
    if (sensor_status.front && sensor_status.right && !sensor_status.left) {
    Serial.println("Left blocked");
    }
    if (!sensor_status.front && !sensor_status.right && sensor_status.left) {
    Serial.println("Front&Right blocked, take left");
    }
    if (!sensor_status.front && sensor_status.right && !sensor_status.left) {
    Serial.println("Front&Left blocked, take right");
    }
    if (sensor_status.front && !sensor_status.right && !sensor_status.left) {
    Serial.println("Right&Left blocked, go straight");
    }

  delay(100); // Wait for 250 milliseconds before reading the sensors again*/

  //--------------------- Motor Driving Loop -------------------------
  //TODO

  /*
  drive straight ahead at the beginning, keep going staright unless the following:

    if front & right is blocked, 
      stop and turn to the left, for am set amount of time (time it takes to rotate the vehicle 90 degrees)
      after rotation, if front is not blocked, continue straight forward, if front still blocked, turn to the left for 200 ms
      keep checking if front is blocked, if not blocked continue forward. If front blocked, turn 
    
    
    if only front is blocked, 
      keep turning to the left until only right is blocked, 
      when only right is blocked, go straight. 

    if only right blocked, 
    just go straight


      */

  // TODO

  // Check if right is continuously blocked for 8 seconds
  if (!sensor_status.right) {
    if (rightBlockedStartTime == 0) { // If not already tracking time
      rightBlockedStartTime = millis(); // Start timing
    } else if (millis() - rightBlockedStartTime >= 5000) { // Check if 8 seconds have elapsed
      drive_left(); // Turn left
      delay(600); // Turn for 0.6 seconds
      rightBlockedStartTime = 0; // Reset the timer
    }
  } else {
    rightBlockedStartTime = 0; // Reset if right is not blocked
  }

  // Drive straight ahead at the beginning
  drive_forward();

  if (!sensor_status.front && !sensor_status.right) {
    // Front and right are blocked
    // stop_motors(); // Stop the motors
    drive_left(); // Start turning to the left
    // delay(1000); // Turn for a set amount of time to rotate 90 degrees
    
    // After rotation
    while (true) {
      if (digitalRead(SENSOR_PIN_1) == HIGH) {
        // If front is not blocked
        drive_forward(); // Continue straight forward
        break; // Exit the while loop
      } else {
        // If front is still blocked
        drive_left(); // Turn to the left for 200 ms
        delay(200); // Delay for 200 ms
      }
    }
  } else if (!sensor_status.front) {
    // Only front is blocked
    while (true) {
      drive_left(); // Keep turning to the left
      if (digitalRead(SENSOR_PIN_1) == HIGH) {
        // When front is no longer blocked
        drive_forward(); // Go straight
        break; // Exit the while loop
      }
    }
  } else if (!sensor_status.right) {
      // Only right is blocked
      drive_forward(); // Just go straight
  }
  
  delay(100); // Short delay to prevent excessive loop cycling
}

/*--------------------------------------------------------------------------*/
/*                      8. Getting Police Car Location                      */
/*----------------------------------------------------------------------- --*/
// #define UDPPORT 2510 // port for game obj transmission
// WiFiUDP UDPServer;
// IPAddress myIPaddress(192, 168, 1, 204); // Andy's IP address.

// // uncomment the router SSID and Password that is being used:
// // const char* ssid     = "TP-Link_05AF";
// // const char* password = "47543454";

// const char* ssid     = "TP-Link_E0C8";
// const char* password = "52665134";

// //const char* ssid     = "TP-Link_FD24"; 
// //const char* password = "65512111";

// void handleUDPServer() {
//    const int UDP_PACKET_SIZE = 14; // can be up to 65535          
//    uint8_t packetBuffer[UDP_PACKET_SIZE];

//    int cb = UDPServer.parsePacket(); // if there is no message cb=0
//    if (cb) {
//       int x, y;
//       packetBuffer[13]=0; // null terminate string
//       UDPServer.read(packetBuffer, UDP_PACKET_SIZE);

//       if()
//       x = atoi((char *)packetBuffer+3); // ##,####,#### 2nd indexed char
//       y = atoi((char *)packetBuffer+8); // ##,####,#### 7th indexed char
//       Serial.print("From Team: ");
//       Serial.println((char *)packetBuffer);
//       Serial.println(x);
//       Serial.println(y);
//    }
// }

// void setup() {
//   Serial.begin(115200);
//   WiFi.begin(ssid, password);
//   WiFi.config(myIPaddress,        // Device IP address.
//       IPAddress(192, 168, 1, 1),   // gateway (not important for 5100).
//       IPAddress(255, 255, 255, 0)); // net mask. 
  
//   UDPServer.begin(UDPPORT);  // 2510 for game arbitrary UDP port# need to use same one.   
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".\n");
//   }
//   Serial.printf("WiFi connected to %s\n", ssid);
//   Serial.print("Using static IP ");
//   Serial.print(myIPaddress); 
//   Serial.print("and UDP port ");
//   Serial.println(UDPPORT);
// }

// void loop() {
//   handleUDPServer();
//   delay(10);
// }
/*****************************************************************************/


