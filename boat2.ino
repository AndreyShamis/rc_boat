#include <Arduino.h>
#include <ESP32Servo.h>

#include "rudder.h"
#include "motor.h"
#include "EncoderHandler.h"
#include "fly_sky.h"


// ==== Global Servo Instances (used in rudder.cpp & motor.cpp) ====
Servo escRight;
Servo escLeft;
Servo servoLeft;
Servo servoRight;

// ==== Global state ====



// ==== Setup ====
void setup() {
    Serial.begin(115200);
  delay(1000);
  // ESCs
  escLeft.setPeriodHertz(50);
  escLeft.attach(21, 1000, 2000);
  escRight.setPeriodHertz(50);
  escRight.attach(5, 1000, 2000);
  setMotorSpeeds(1000, 1000);  // Arm ESCs
  // Rudders
  servoLeft.setPeriodHertz(50);
  servoRight.setPeriodHertz(50);
  servoLeft.attach(18, 1000, 2000);
  servoRight.attach(19, 1000, 2000);
  delay(1000);


  setRudderTrim(0, 0);
  setRudderAngle(0);
  initEncoder();
  Serial.println("System Ready: ESC x2 + Rudders Initialized");
  setRudderAngle(0);
  delay(1);
  updateEncoder();
  setEncoderMode(MODE_RUDDER);
  setRudderAngles(25, -25);
  delay(100);
  setRudderAngles(-25, 25);
  delay(100);
  setRudderAngle(0);
delay(100);
setupFlySkyPPM();
}

void setMotorPower(int percent) {
  setMotor(percent, percent);  // оба мотора с одинаковой мощностью
}


// ==== Loop ====
void loop() {
  updateEncoder();

uint16_t ch1 = getFlySkyChannel(0); // rudder
uint16_t ch2 = getFlySkyChannel(2); // throttle
  // Преобразуем в понятные значения
  int rudder = map(ch1, 1000, 2000, -60, 60);   // управление рулём
  int throttle = map(ch2, 1000, 2000, 0, 100);  // мощность мотора (0-100%)

  // Применяем
  setRudderAngle(rudder);
  setMotorPower(throttle);
// static uint32_t lastCounter = 0;
// if (ppmCounter != lastCounter) {
//   Serial.print("PPM ISR Triggered: ");
//   Serial.println(ppmCounter);
//   lastCounter = ppmCounter;
// }

  delay(5);
}
