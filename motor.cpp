#include <Arduino.h>
#include <ESP32Servo.h>
#include "motor.h"

extern Servo escLeft;
extern Servo escRight;
int currentLeft;
int currentRight;

void setMotorSpeeds(int leftThrottle, int rightThrottle) {
  escLeft.writeMicroseconds(constrain(leftThrottle, 1000, 2000));
  escRight.writeMicroseconds(constrain(rightThrottle, 1000, 2000));
}

int mapWithDeadzone(int percent) {
  ushort broker = 1100;
  percent = constrain(percent, 0, 100);
  if (percent == 0) return 1000;
  else if (percent == 1) return broker;
  else return map(percent, 1, 100, broker, 2000);
}

void setMotor(int leftPercent, int rightPercent) {
  leftPercent = constrain(leftPercent, 0, 100);
  rightPercent = constrain(rightPercent, 0, 100);

  if (leftPercent < currentLeft || rightPercent < currentRight) {
    int leftPulse = mapWithDeadzone(leftPercent);
    int rightPulse = mapWithDeadzone(rightPercent);

    setMotorSpeeds(leftPulse, rightPulse);
    // if (Serial && Serial.availableForWrite()) {
    //   Serial.printf("MsetMotorSpeeds: No step %d µs, Right: %d µs - %d-%d\n", leftPulse, rightPulse, leftPercent, rightPercent);
    // }
    currentLeft = leftPercent;
    currentRight = rightPercent;
    return;
  }
  int deltaLeft = abs(leftPercent - currentLeft);
  int deltaRight = abs(rightPercent - currentRight);

  const int baseSteps = 5;
  const int fastStepThreshold = 10;  // 10% в "процентах", не в микросекундах
  int steps = (deltaLeft < fastStepThreshold && deltaRight < fastStepThreshold) ? 1 : baseSteps;

  for (int i = 1; i <= steps; i++) {
    float factor = pow((float)i / steps, 1.8);
    int newLeft = currentLeft + (leftPercent - currentLeft) * factor;
    int newRight = currentRight + (rightPercent - currentRight) * factor;

    int leftPulse = mapWithDeadzone(newLeft);
    int rightPulse = mapWithDeadzone(newRight);

    setMotorSpeeds(leftPulse, rightPulse);
    // if (Serial && Serial.availableForWrite()) {
    //   Serial.printf("MsetMotorSpeeds: STEP %d %d µs, Right: %d µs  %d - %d\n", i, leftPulse, rightPulse, leftPercent, rightPercent);
    // }
    delay(1);
  }

  currentLeft = leftPercent;
  currentRight = rightPercent;
}