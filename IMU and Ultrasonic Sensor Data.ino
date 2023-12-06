// Program for GY-85 IMU Sensor and HC-SR04 Ultrasonic Sensor (Head Movement)
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_HMC5883_U.h>
#include "ITG3200.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
ITG3200 gyro;
float pitch = 0.0;
float yaw = 0.0;
float yawOffset = 0.0;
const int trigPin = 9;
const int echoPin = 10;
long duration;
float distance;
float initialDistance = 0.0; // Initial distance from the ultrasonic sensor

void setup() {
  Serial.begin(19200);
  accel.begin();
  mag.begin();
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  // Calibration Routine
  for (int i = 0; i < 100; i++) {
    sensors_event_t magEvent;
    mag.getEvent(&magEvent);
    float heading = atan2(magEvent.magnetic.y, magEvent.magnetic.x);
    if (heading < 0) {
      heading += 2 * PI;
    }
    yawOffset += heading;
    delay(10);
  }
  yawOffset /= 100;

  // Initial distance measurement
  initialDistance = measureDistance();
}

float measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t event;
  accel.getEvent(&event);
  float pitchAcc = -atan2(event.acceleration.y, event.acceleration.z) * RAD_TO_DEG;
  pitchAcc = constrain(pitchAcc, -90, 90);
  pitchAcc = round(pitchAcc);
  pitchAcc += 1;

  sensors_event_t magEvent;
  mag.getEvent(&magEvent);

  float heading = atan2(magEvent.magnetic.y, magEvent.magnetic.x);
  if (heading < 0) {
    heading += 2 * PI;
  }
  float yawAcc = (heading - yawOffset) * RAD_TO_DEG;
  // Map yaw values if they are more than 300
  if (yawAcc > 300) {
    yawAcc = map(yawAcc, 360, 300, -40, -90);
  }
  yawAcc = round(yawAcc);

  float rollAcc = event.acceleration.x;
  rollAcc = map(rollAcc, -10, 10, -90, 90);
  rollAcc = round(rollAcc);

  // Measure the current distance
  float currentDistance = measureDistance();

  // Calculate the difference in distance traveled
  float distanceDifference = currentDistance - initialDistance;
  distanceDifference = round(distanceDifference);

  Serial.print(distanceDifference);
  Serial.print(",");
  Serial.print(yawAcc);
  Serial.print(",");
  Serial.print(rollAcc);
  //Serial.print(",");
  //Serial.print(pitchAcc);
  Serial.println(" ");
}
