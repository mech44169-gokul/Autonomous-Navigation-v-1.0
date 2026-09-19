# Autonomous-Navigation-v-1.0
Esp 32 version 1.0
#include "BluetoothSerial.h"
#include <TinyGPS++.h>

// Pin Definitions based on Circuit Diagram
#define LEFT_LINE_PIN   34   // Left TCRT5000 Analog
#define RIGHT_LINE_PIN  35   // Right TCRT5000 Analog

#define TRIG_PIN        15   // HC-SR04 Trig
#define ECHO_PIN        13   // HC-SR04 Echo (via voltage divider)

#define GPS_RX_PIN      16   // ESP32 RX2 connected to GPS TX
#define GPS_TX_PIN      17   // ESP32 TX2 connected to GPS RX

// L298N Motor Driver Pins
#define IN1             23   // Motor Left Forward
#define IN2             19   // Motor Left Backward
#define IN3             18   // Motor Right Forward
#define IN4             5    // Motor Right Backward

// Thresholds & Constants
#define LINE_THRESHOLD  2000 // ADC value for black line (adjust based on calibration)
#define STOP_DISTANCE   2.0  // Stop distance in centimeters
#define GPS_TOLERANCE   3.0  // Target arrival threshold in meters

// Modes
enum DriveMode { MODE_STOP, MODE_PATH, MODE_GPS };
DriveMode currentMode = MODE_STOP;

// Instances
BluetoothSerial SerialBT;
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);

// Target Waypoint Coordinates
double targetLat = 0.0;
double targetLng = 0.0;
bool targetSet = false;

void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  SerialBT.begin("ESP32_Robot_Car"); // Bluetooth Device Name

  pinMode(LEFT_LINE_PIN, INPUT);
  pinMode(RIGHT_LINE_PIN, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();
  Serial.println("System Ready. Connect via Bluetooth.");
}

void loop() {
  handleBluetooth();
  
  // Read incoming GPS stream
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }

  // Safety obstacle check
  float distance = getUltrasonicDistance();
  if (distance > 0 && distance <= STOP_DISTANCE) {
    stopMotors();
    return;
  }

  // Execute active mode
  switch (currentMode) {
    case MODE_PATH:
      followLine();
      break;

    case MODE_GPS:
      navigateGPS();
      break;

    case MODE_STOP:
    default:
      stopMotors();
      break;
  }
}

// ---------------- Helper Functions ---------------- //

float getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000); // 25ms timeout (~4m max)
  if (duration == 0) return 999.0;
  return (duration * 0.0343) / 2.0;
}

void handleBluetooth() {
  if (!SerialBT.available()) return;

  String cmd = SerialBT.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "path") {
    currentMode = MODE_PATH;
    SerialBT.println("Mode: Line Following Started");
  } 
  else if (cmd.startsWith("gps")) {
    // Expected format: "gps <latitude> <longitude>"
    // Example: "gps 11.3934 79.7123"
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);

    if (firstSpace != -1 && secondSpace != -1) {
      targetLat = cmd.substring(firstSpace + 1, secondSpace).toDouble();
      targetLng = cmd.substring(secondSpace + 1).toDouble();
      targetSet = true;
      currentMode = MODE_GPS;
      SerialBT.println("Mode: GPS Navigation Started to Target");
    } else {
      SerialBT.println("Error: Send GPS target as: gps <lat> <lng>");
    }
  } 
  else if (cmd == "stop") {
    currentMode = MODE_STOP;
    stopMotors();
    SerialBT.println("Mode: Stopped");
  }
}

void followLine() {
  int leftVal = analogRead(LEFT_LINE_PIN);
  int rightVal = analogRead(RIGHT_LINE_PIN);

  bool leftBlack = (leftVal > LINE_THRESHOLD);
  bool rightBlack = (rightVal > LINE_THRESHOLD);

  if (leftBlack && rightBlack) {
    moveForward();
  } else if (leftBlack && !rightBlack) {
    turnLeft();
  } else if (!leftBlack && rightBlack) {
    turnRight();
  } else {
    moveForward();
  }
}

void navigateGPS() {
  if (!targetSet) {
    stopMotors();
    return;
  }

  if (!gps.location.isValid()) {
    stopMotors(); // Wait for valid GPS satellite lock
    return;
  }

  double currentLat = gps.location.lat();
  double currentLng = gps.location.lng();

  double distanceToTarget = TinyGPSPlus::distanceBetween(currentLat, currentLng, targetLat, targetLng);
  double targetHeading = TinyGPSPlus::courseTo(currentLat, currentLng, targetLat, targetLng);

  if (distanceToTarget <= GPS_TOLERANCE) {
    stopMotors();
    currentMode = MODE_STOP;
    SerialBT.println("Target Destination Reached!");
    targetSet = false;
    return;
  }

  // Steer towards target waypoint using GPS heading
  double currentHeading = gps.course.deg();
  double headingError = targetHeading - currentHeading;

  if (headingError < -180) headingError += 360;
  if (headingError > 180) headingError -= 360;

  if (abs(headingError) < 20) {
    moveForward();
  } else if (headingError < 0) {
    turnLeft();
  } else {
    turnRight();
  }
}

// ---------------- Motor Primitives ---------------- //

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
