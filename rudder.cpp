#include <Arduino.h>
#include <ESP32Servo.h>
#include "rudder.h"

extern Servo servoLeft;
extern Servo servoRight;
int leftTrim = 0;
int rightTrim = 0;

void setRudderTrim(int leftDeg, int rightDeg) {
  leftTrim = constrain(leftDeg, -90, 90);
  rightTrim = constrain(rightDeg, -90, 90);

  // if (Serial) {
  //   Serial.printf("Set trim - Left: %d°, Right: %d°\n", leftTrim, rightTrim);
  // }
}

void setRudderAngles(int leftAngle, int rightAngle) {
  leftAngle = constrain(leftAngle, -90, 90);
  rightAngle = constrain(rightAngle, -90, 90);
  int center = 1500;

  int leftOffset = map(leftAngle, -90, 90, -500, 500);
  int rightOffset = map(rightAngle, -90, 90, -500, 500);

  int leftPulse = center + leftOffset + map(leftTrim, -90, 90, -500, 500);
  int rightPulse = center + rightOffset + map(rightTrim, -90, 90, -500, 500);

  leftPulse = constrain(leftPulse, 500, 2500);
  rightPulse = constrain(rightPulse, 500, 2500);

  servoLeft.writeMicroseconds(leftPulse);
  servoRight.writeMicroseconds(rightPulse);

  // if (Serial && Serial.availableForWrite()) {
  //   Serial.printf("Rudder L: %d° (%d µs), R: %d° (%d µs)\n",
  //                 leftAngle, leftPulse, rightAngle, rightPulse);
  // }
}

void setRudderAngle(int angle) {
  setRudderAngles(angle, angle);
}
