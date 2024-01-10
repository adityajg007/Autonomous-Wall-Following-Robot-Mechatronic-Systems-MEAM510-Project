/* Name: MEAM510-Final-Project.ino
 * MEAM5100 Final Project
 * Team: Aditya, Andy and Vaibhav.
 * Instructions:
   Uncomment the code to run the different parts of the lab.
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
*/
#include <stdint.h>
#include <utility>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "html510.h" // HTML library for UDP communication.
#include "vive510.h" // Vive library for HTC Vive communication.
#define BAUD_RATE 115200
/********************** Include necessary libraries above *************************/

/*------------------------------------------------------------------------*/
/*                               Wall Following                           */
/*------------------------------------------------------------------------*/
// /*
//-------------------- Define Motor Pin --------------------------
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

// --------------------- Setup -------------------------
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

//--------------------- Motor Driving Functions -------------------------
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 100) {
  uint32_t duty = LEDC_RESOLUTION * min(value, valueMax) / valueMax;
  ledcWrite(channel, duty);
}

void drive_forward() {
  
  ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.5); // 0.5 to reduce speed for both
  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.5); // 0.78 correction to go straight

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
// Variable to store the time when right sensor first gets blocked:
unsigned long rightBlockedStartTime = 0; 

//--------------------- Main Loop -------------------------
void loop() {
  //--------------------- IR Loop -------------------------
  // Read the sensor values
  SensorStatus sensor_status;
  sensor_status.front = digitalRead(SENSOR_PIN_1) == HIGH;
  sensor_status.right = digitalRead(SENSOR_PIN_2) == HIGH;
  sensor_status.left = digitalRead(SENSOR_PIN_3) == HIGH;

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
// */
/*****************************************************************************/

/*------------------------------------------------------------------------*/
/*                             Beacon Tracking                            */
/*------------------------------------------------------------------------*/
// /*
//******** Definitions *********
//-------------------- Define Motor Pin --------------------------
#define LEDC_CHANNEL_LEFT 1 // use the first channel for the left motor.
#define LEDC_CHANNEL_RIGHT 0 // use the second channel for the right motor.
#define LEDC_RESOLUTION_BITS 14
#define LEDC_RESOLUTION  ((1 << LEDC_RESOLUTION_BITS) - 1)
#define LEDC_FREQ_HZ 1000 // greater than 39000/2^14 = 2.3 Hz

// Motor Right:
#define RIGHTMOTOR_PIN 33 // PWM pin for motor.
#define RIGHTMOTORD1_PIN 19 // Direction pin 1 for motor.
#define RIGHTMOTORD2_PIN 21 // Direction pin 2 for motor.

// Motor Left:
#define LEFTMOTOR_PIN 17 // PWM pin for motor.
#define LEFTMOTORD1_PIN 16 // Direction pin 1 for motor.
#define LEFTMOTORD2_PIN 14 // Direction pin 2 for motor.

#define DUTY 100 // Duty cycle in % for PWM.

//--------------------- Define IR Pin -------------------------
// Define the pins where the sensors are connected:
#define SENSOR_PIN_1 35 // Front IR
#define SENSOR_PIN_2 10 // Right IR
#define SENSOR_PIN_3 36 // Left IR

//--------------------- Define Beacon Pin -------------------------
#define BEACON_PIN 3  // Pin to which the signal is connected

// Structure to hold the status of all sensors:
struct SensorStatus {
  bool front;
  bool right;
  bool left;
};

unsigned long rightBlockedStartTime = 0; // time when right sensor first gets blocked.
uint16_t BeaconFreq = 0; // Frequency of the beacon.

//******* Setup ********
void setup() {
  //--------------------- IR-Retro Reflective Setup -------------------------
  Serial.begin(115200); // Start the serial communication at 115200 baud rate.

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

  //--------------------- Beacon Setup -------------------------
  pinMode(BEACON_PIN, INPUT);

  //--------------------- LED Setup -------------------------
  pinMode(LED_PIN, OUTPUT); // Initialize the LED pin as an output
}

//--------------------- Motor Driving Functions -------------------------
// can set syntax to be like analogWrite() with input[ 0 : valueMax ]:
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 100) {
  uint32_t duty = LEDC_RESOLUTION * min(value, valueMax) / valueMax;
  ledcWrite(channel, duty);
}

void drive_forward() {
  ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.5); // 0.5 to reduce speed for both
  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.5); // 0.78 correction to go straight

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
  ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.6);
  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.6); // 0.78 correction to go straight.

  digitalWrite(LEFTMOTORD1_PIN, HIGH);
  digitalWrite(LEFTMOTORD2_PIN, LOW);

  digitalWrite(RIGHTMOTORD1_PIN, LOW);
  digitalWrite(RIGHTMOTORD2_PIN, HIGH);
}

void drive_right() {
  ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.5); // 0.5 to reduce speed.
  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.5);

  digitalWrite(LEFTMOTORD1_PIN, LOW);
  digitalWrite(LEFTMOTORD2_PIN, HIGH);

  digitalWrite(RIGHTMOTORD1_PIN, HIGH);
  digitalWrite(RIGHTMOTORD2_PIN, LOW);
}

void stop() {
  ledcAnalogWrite(LEDC_CHANNEL_LEFT, 0);
  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, 0);

  digitalWrite(LEFTMOTORD1_PIN, LOW);
  digitalWrite(LEFTMOTORD2_PIN, HIGH);

  digitalWrite(RIGHTMOTORD1_PIN, HIGH);
  digitalWrite(RIGHTMOTORD2_PIN, LOW);
}

uint16_t detectFrequency() {
  unsigned long OnTime = pulseIn(BEACON_PIN, HIGH);  // Measure on the period of the incoming signal
  if (OnTime > 400 && OnTime < 3000) {
    Serial.print("550Hz detected\n");
    return 550;
  } else if (OnTime > 8000) {
    Serial.print("23Hz detected\n");
    return 23;
  } else {
    Serial.print("Nothing detected\n");
    return 0;
  }
}

void headToBeacon() {
  if (BeaconFreq == 550 || BeaconFreq == 23) { 
    drive_forward();
    Serial.print("Driving forward\n");
    delay(1000);
    // return; // Exit the function after waiting for 6 seconds
  } else {
    digitalWrite(LED_PIN, HIGH); // Turn on the LED
    drive_left();
    delay(100);
    stop();
    Serial.print("Driving left\n");
    delay(1000);
  }
  BeaconFreq = detectFrequency();
}

//************ Loop *************
void loop() {

  //--------------------- Beacon Loop -------------------------
  BeaconFreq = detectFrequency();
  while (true) {
    headToBeacon();
  }
}
// */
/*****************************************************************************/

/*------------------------------------------------------------------------*/
/*                          Robot Vive Location                           */
/*------------------------------------------------------------------------*/
// /*
#define RGBLED 2 // for ESP32S2 Devkit pin 18, for M5 stamp=2
#define SIGNALPIN1 26 // pin receiving signal from Vive circuit
#define UDPPORT 2510 // For GTA 2022C game 
#define STUDENTIP 204 // Andy's IP number
#define teamNumber 31 
#define FREQ 1 // in Hz

const char* ssid     = "TP-Link_E0C8";
const char* password = "52665134";

Vive510 vive1(SIGNALPIN1);

WiFiUDP UDPTestServer;
IPAddress ipTarget(192, 168, 1, 255); // 255 => broadcast

void UdpSend(int x, int y) {
  char udpBuffer[20];
  sprintf(udpBuffer, "%02d:%4d,%4d",teamNumber,x,y);                                                
  UDPTestServer.beginPacket(ipTarget, UDPPORT);
  UDPTestServer.println(udpBuffer);
  UDPTestServer.endPacket();
  Serial.println(udpBuffer);
}
               
void setup() {
  int i = 0;
  Serial.begin(115200);

  WiFi.mode(WIFI_AP_STA);
  WiFi.config(IPAddress(192, 168, 1, STUDENTIP), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);

  Serial.printf("team  #%d ", teamNumber); 
  Serial.print("Connecting to ");  Serial.println(ssid);
  while(WiFi.status()!=WL_CONNECTED && i++ < 20){
    delay(500);   Serial.print(".");
  }
  if (i < 19) {
    Serial.println("WiFi connected as "); Serial.print(WiFi.localIP());
  } else {
    Serial.printf("Could not connect err: %d ",i); 
  }
  UDPTestServer.begin(UDPPORT);
  vive1.begin();
  Serial.println("Vive trackers started");
}
                                 
void loop() {  
  static long int ms = millis();
  static uint16_t x,y;

  if (millis()-ms>1000/FREQ) {
    ms = millis();
    if (WiFi.status()==WL_CONNECTED)
        neopixelWrite(RGBLED,255,255,255);  // full white
    UdpSend(x,y);
  }
  
  if (vive1.status() == VIVE_RECEIVING) {
    x = vive1.xCoord();
    y = vive1.yCoord();
    Serial.printf("X %d, Y %d\n", vive1.xCoord(), vive1.yCoord());
    neopixelWrite(RGBLED, 0, x/200, y/200);  // blue to greenish
  }
  else {
    x=0;
    y=0; 
    switch (vive1.sync(5)) {
      break;
      case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
        neopixelWrite(RGBLED,64,32,0);  // yellowish
      break;
      default:
      case VIVE_NO_SIGNAL: // nothing detected     
        neopixelWrite(RGBLED,128,0,0);  // red
    }
  }
    
  delay(20);
}
// */
/*****************************************************************************/



/*------------------------------------------------------------------------*/
/*                          Pushing Police Car                            */
/*------------------------------------------------------------------------*/
// /*
// Motor Pins:
#define LEDC_CHANNEL_LEFT 1 // use the first channel for the left motor.
#define LEDC_CHANNEL_RIGHT 0 // use the second channel for the right motor.
#define LEDC_RESOLUTION_BITS 14
#define LEDC_RESOLUTION  ((1 << LEDC_RESOLUTION_BITS) - 1)
#define LEDC_FREQ_HZ 1000 // greater than 39000/2^14 = 2.3 Hz

// Motor Right:
#define RIGHTMOTOR_PIN 33 // PWM pin for motor.
#define RIGHTMOTORD1_PIN 19 // Direction pin 1 for motor.
#define RIGHTMOTORD2_PIN 21 // Direction pin 2 for motor.

// Motor Left:
#define LEFTMOTOR_PIN 17 // PWM pin for motor.
#define LEFTMOTORD1_PIN 16 // Direction pin 1 for motor.
#define LEFTMOTORD2_PIN 14 // Direction pin 2 for motor.

#define DUTY 100 // Duty cycle in % for PWM.

// Define the pins where the sensors are connected:
#define SENSOR_PIN_1 35 // Front IR
#define SENSOR_PIN_2 10 // Right IR
#define SENSOR_PIN_3 36 // Left IR

#define UDPPORT 2510 // port for game obj transmission
#define FREQ 1 // in Hz

WiFiUDP UDPServer;
IPAddress myIPaddress(192, 168, 1, 204); // Andy's IP address.

const char* ssid     = "TP-Link_E0C8";
const char* password = "52665134";
int Police_x = 0; // Global variable to store Police Car x coordinate
int Police_y = 0; // Global variable to store Police Car y coordinate

// Structure to hold the status of all sensors:
struct SensorStatus {
  bool front;
  bool right;
  bool left;
};

// Setup function:
void setup() {
  Serial.begin(115200); // Start the serial communication at 115200 baud rate.
  // Set the sensor pins as input:
  pinMode(SENSOR_PIN_1, INPUT);
  pinMode(SENSOR_PIN_2, INPUT);
  pinMode(SENSOR_PIN_3, INPUT);

  //--------------------- Motor Setup -------------------------
  // Left:
  pinMode(LEFTMOTOR_PIN, OUTPUT);
  pinMode(LEFTMOTORD1_PIN, OUTPUT);
  pinMode(LEFTMOTORD2_PIN, OUTPUT);

  ledcSetup(LEDC_CHANNEL_LEFT, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttachPin(LEFTMOTOR_PIN, LEDC_CHANNEL_LEFT);

  // Right:
  pinMode(RIGHTMOTOR_PIN, OUTPUT);
  pinMode(RIGHTMOTORD1_PIN, OUTPUT);
  pinMode(RIGHTMOTORD2_PIN, OUTPUT);

  ledcSetup(LEDC_CHANNEL_RIGHT, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttachPin(RIGHTMOTOR_PIN, LEDC_CHANNEL_RIGHT);

  vive1.begin();
  Serial.println("Vive trackers started");
  WiFi.begin(ssid, password);
  WiFi.config(myIPaddress,          // Device IP address.
      IPAddress(192, 168, 1, 1),    // gateway (not important for 5100).
      IPAddress(255, 255, 255, 0)); // net mask. 
  UDPServer.begin(UDPPORT);  // 2510 for game arbitrary UDP port# need to use same one.   
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".\n");
  }
}


//--------------------- Motor Driving Functions -------------------------
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 100) {
  uint32_t duty = LEDC_RESOLUTION * min(value, valueMax) / valueMax;
  ledcWrite(channel, duty);
}

void drive_forward() {
  
  ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY*1);
  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY*1);

  digitalWrite(LEFTMOTORD1_PIN, LOW);
  digitalWrite(LEFTMOTORD2_PIN, HIGH);

  digitalWrite(RIGHTMOTORD1_PIN, LOW);
  digitalWrite(RIGHTMOTORD2_PIN, HIGH);
}

void drive_backward() {
  
  ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.5); // 0.5 to reduce speed for both
  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.5); // correction to go straight

  digitalWrite(LEFTMOTORD1_PIN, HIGH);
  digitalWrite(LEFTMOTORD2_PIN, LOW);

  digitalWrite(RIGHTMOTORD1_PIN, HIGH);
  digitalWrite(RIGHTMOTORD2_PIN, LOW);
}

void drive_left() {

  ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.5);
  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.5); // correction to go straight

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

void stop() {
  ledcAnalogWrite(LEDC_CHANNEL_LEFT, 0);
  ledcAnalogWrite(LEDC_CHANNEL_RIGHT, 0);

  digitalWrite(LEFTMOTORD1_PIN, LOW);
  digitalWrite(LEFTMOTORD2_PIN, HIGH);

  digitalWrite(RIGHTMOTORD1_PIN, HIGH);
  digitalWrite(RIGHTMOTORD2_PIN, LOW);
}


void handleUDPServer() {
   const int UDP_PACKET_SIZE = 14; // can be up to 65535          
   uint8_t packetBuffer[UDP_PACKET_SIZE];

   int cb = UDPServer.parsePacket(); // if there is no message cb=0
   if (cb) {
      packetBuffer[13] = 0; // null terminate string
      UDPServer.read(packetBuffer, UDP_PACKET_SIZE);
      Police_x = atoi((char *)packetBuffer + 3); // ##,####,#### 2nd indexed char
      Police_y = atoi((char *)packetBuffer + 8); // ##,####,#### 7th indexed char
   }
}

// Function to get the Vive coordinates:
struct ViveCoordinates {
  uint16_t Robot_x;
  uint16_t Robot_y;
};
ViveCoordinates getViveCoordinates(Vive510& vive) {
  ViveCoordinates coordinates;
  coordinates.Robot_x = vive.xCoord();
  coordinates.Robot_y = vive.yCoord();
  return coordinates;
}

void loop() {
  SensorStatus sensor_status;
  sensor_status.front = digitalRead(SENSOR_PIN_1) == HIGH;
  sensor_status.right = digitalRead(SENSOR_PIN_2) == HIGH;
  sensor_status.left = digitalRead(SENSOR_PIN_3) == HIGH;

  static long int ms = millis();
  static uint16_t Robot_x, Robot_y;
  if (millis() - ms > 1000 / FREQ) {
    ms = millis();
  }

  // Main conrol loop to reach the Police Car:
  if (vive1.status() == VIVE_RECEIVING) {
    ViveCoordinates viveCoordinates = getViveCoordinates(vive1);
    Robot_x = viveCoordinates.x;
    Robot_y = viveCoordinates.y;
    handleUDPServer();
    // Control loop to reach the Police Car:
    if (Robot_y > Police_y && Robot_x > Police_x) {
      delay(500);
      drive_left();
      delay(200);
      stop();
      delay(500);
      drive_forward();
      delay(500);
      stop();
      delay(500);
    } else if (Robot_y < Police_y && Robot_x > Police_x) {
      delay(500);
      drive_right();
      delay(200);
      stop();
      delay(500);
      drive_forward();
      delay(500);
      stop();
      delay(500);
    } else if (Robot_y > Police_y) {
      drive_forward();
    } else if (Robot_y < Police_y) {
      drive_backward();
    } else {
      stop();
    }
  } else {
    x = 0;
    y = 0;
    switch (vive1.sync(5)) {
      case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
        neopixelWrite(RGBLED, 64, 32, 0);  // yellowish
        break;
      default:
      case VIVE_NO_SIGNAL: // nothing detected
        neopixelWrite(RGBLED, 128, 0, 0);  // red
    }
  }
  delay(20);
}
// */
/*****************************************************************************/