
/*#define SENSOR_PIN 32 // Define the pin where the sensor is connected

void setup() {
  Serial.begin(115200);      // Start the serial communication at 115200 baud rate
  pinMode(SENSOR_PIN, INPUT); // Set the sensor pin as input
}

void loop() {
  int sensorValue = digitalRead(SENSOR_PIN); // Read the sensor value

  // Check if the sensor is detecting an object
  if(sensorValue == HIGH) {
    Serial.println(" No Object Detected :(");
  } else {
    Serial.println("Object Detected :)");
  }
  delay(500); // Wait for 500 milliseconds before reading the sensor again
}*/

/** Define the pins where the sensors are connected
#define SENSOR_PIN_1 32
#define SENSOR_PIN_2 35
#define SENSOR_PIN_3 34
#define SENSOR_PIN_4 39

void setup() {
  Serial.begin(115200); // Start the serial communication at 115200 baud rate

  // Set the sensor pins as input
  pinMode(SENSOR_PIN_1, INPUT);
  pinMode(SENSOR_PIN_2, INPUT);
  pinMode(SENSOR_PIN_3, INPUT);
  pinMode(SENSOR_PIN_4, INPUT);
}

void loop() {
  // Read the sensor values
  int sensorValue1 = digitalRead(SENSOR_PIN_1);
  int sensorValue2 = digitalRead(SENSOR_PIN_2);
  int sensorValue3 = digitalRead(SENSOR_PIN_3);
  int sensorValue4 = digitalRead(SENSOR_PIN_4);

  // Check and print the status of each sensor
  Serial.print("Front Sensor: "); // 6 inches threshold, 1 dash
  Serial.println(sensorValue1 == HIGH ? "No Object Detected :(" : "Object Detected :)"); 
  Serial.print("Right Seneor: "); // 2.5 inch, 2 dashes
  Serial.println(sensorValue2 == HIGH ? "No Object Detected :(" : "Object Detected :)");
  Serial.print("Left Sensor: "); // 2.5 inch, 3 dashes
  Serial.println(sensorValue3 == HIGH ? "No Object Detected :(" : "Object Detected :)");
  Serial.print("Back Sensor: "); // 2 inches, 4 dashes
  Serial.println(sensorValue4 == HIGH ? "No Object Detected :(" : "Object Detected :)");

  delay(500); // Wait for 500 milliseconds before reading the sensors again
}**/

//----------------------------------------------
// Define the pins where the sensors are connected
#define SENSOR_PIN_1 35
#define SENSOR_PIN_2 10
#define SENSOR_PIN_3 36

// Structure to hold the status of all sensors
struct SensorStatus {
  bool front;
  bool right;
  bool left;
};

void setup() {
  Serial.begin(115200); // Start the serial communication at 115200 baud rate

  // Set the sensor pins as input
  pinMode(SENSOR_PIN_1, INPUT);
  pinMode(SENSOR_PIN_2, INPUT);
  pinMode(SENSOR_PIN_3, INPUT);
}

void loop() {
  // Read the sensor values
  SensorStatus sensor_status;
  sensor_status.front = digitalRead(SENSOR_PIN_1) == HIGH;
  sensor_status.right = digitalRead(SENSOR_PIN_2) == HIGH;
  sensor_status.left = digitalRead(SENSOR_PIN_3) == HIGH;

  // Check and print the status of each sensor
  /**Serial.print("Front Sensor: ");
  Serial.println(sensor_status.front ? "No Object Detected :(" : "Object Detected :)");
  Serial.print("Right Sensor: ");
  Serial.println(sensor_status.right ? "No Object Detected :(" : "Object Detected :)");
  Serial.print("Left Sensor: ");
  Serial.println(sensor_status.left ? "No Object Detected :(" : "Object Detected :)");*/

  // Use the sensor_status variable in an if-else conditional
  if (sensor_status.front && sensor_status.right && sensor_status.left) {
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

  delay(500); // Wait for 500 milliseconds before reading the sensors again
}