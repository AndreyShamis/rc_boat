#include <Arduino.h>
#include <ESP32Servo.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
// #include <DFRobot_QMC5883.h>
#include <ESP32Servo.h>

#include "rudder.h"
#include "motor.h"
#include "EncoderHandler.h"
#include "fly_sky.h"

#define COMPASS_I2C_SDA 21  // Compass I2C
#define COMPASS_I2C_SCL 22  // Compass I2C
#define GPS_RX 25           // GPS UART
#define GPS_TX 26           // GPS UART
#define MOTOR_LEFT 17       // ESC
#define MOTOR_RIGHT 5       // ESC
#define RUDDER_LEFT 18      // Servo
#define RUDDER_RIGHT 19     // Servo



TinyGPSPlus gps;
HardwareSerial GPSserial(2);  // Serial2 for GPS
// DFRobot_QMC5883 compass;

Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);


// ==== Global Servo Instances (used in rudder.cpp & motor.cpp) ====
Servo escRight;
Servo escLeft;
Servo servoLeft;
Servo servoRight;

// ==== Global state ====



// ==== Setup ====
void setup() {
  Serial.begin(115200);
  Wire.begin(COMPASS_I2C_SDA, COMPASS_I2C_SCL);  // SDA, SCL for compass
  Serial.println("Scanning I2C bus...");

  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at 0x");
      Serial.println(address, HEX);
    }
    delay(10);
  }


  if (!compass.begin()) {
    Serial.println("HMC5883L not detected. Check wiring!");
    while (1)
      ;
  }
  Serial.println("HMC5883L ready!");

  GPSserial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);  // GPS UART

  delay(1000);
  // ESCs
  escLeft.setPeriodHertz(50);
  escLeft.attach(MOTOR_LEFT, 1000, 2000);
  escRight.setPeriodHertz(50);
  escRight.attach(MOTOR_RIGHT, 1000, 2000);
  setMotorSpeeds(1000, 1000);  // Arm ESCs
  // Rudders
  servoLeft.setPeriodHertz(50);
  servoRight.setPeriodHertz(50);
  servoLeft.attach(RUDDER_LEFT, 1000, 2000);
  servoRight.attach(RUDDER_RIGHT, 1000, 2000);
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
  setupFlySkyPPM();
  delay(10);

  // ==== GPS & Compass Init ====

  // while (!compass.begin()) {
  //   Serial.println("Compass not found");
  //   delay(1000);
  // }
  // compass.setRange(QMC5883_RANGE_2GA);
  // compass.setMeasurementMode(QMC5883_CONTINOUS);
  // compass.setDataRate(QMC5883_DATARATE_50HZ);
  // compass.setSamples(QMC5883_SAMPLES_8);
  // Vector norm = compass.readNormalize();
  // Serial.print("X: "); Serial.print(norm.XAxis);
  // Serial.print(" Y: "); Serial.print(norm.YAxis);
  // Serial.print(" Z: "); Serial.println(norm.ZAxis);
  // Serial.println("GPS and Compass Ready.");
  delay(100);
}

void setMotorPower(int percent) {
  setMotor(percent, percent);  // оба мотора с одинаковой мощностью
}


// ==== Loop ====
void loop() {
  updateEncoder();

  uint16_t ch1 = getFlySkyChannel(0);  // rudder
  uint16_t ch2 = getFlySkyChannel(2);  // throttle
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
  static unsigned long lastStatusTime = 0;
static unsigned long lastHeadingTime = 0;

  // ==== Read GPS ====
  // Показать "сырые" данные от GPS модуля
  while (GPSserial.available()) {
    char c = GPSserial.read();
    if (millis() - lastStatusTime >= 15000) {
      Serial.write(c);  // покажет NMEA строки
    }
    gps.encode(c);  // передаём в TinyGPS++
  }

  // Каждые 15 секунд
  if (millis() - lastStatusTime >= 15000) {
    lastStatusTime = millis();

    if (gps.location.isValid()) {
      Serial.println("✅ GPS FIX:");
      Serial.print("Latitude: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 6);
    } else {
      Serial.println("🔄 Waiting for GPS fix...");
      if (gps.satellites.isValid()) {
        Serial.print("Satellites in view: ");
        Serial.println(gps.satellites.value());
      } else {
        Serial.println("Satellites info not yet available.");
      }
      if (gps.altitude.isValid()) {
        Serial.print("Altitude: ");
        Serial.print(gps.altitude.meters());
        Serial.println(" m");
      }
      if (gps.speed.isValid()) {
        Serial.print("Speed: ");
        Serial.print(gps.speed.kmph());
        Serial.println(" km/h");
      }
    }

    //     // ==== Read Compass ====
    // sVector_t mag = compass.readRaw();
    // float heading = atan2(mag.YAxis, mag.XAxis);
    // if (heading < 0) heading += 2 * PI;
    // float headingDegrees = heading * 180 / PI;
    // Serial.print("Heading: ");
    // Serial.print(headingDegrees);
    // Serial.println("°");
    sensors_event_t event;
    compass.getEvent(&event);

    Serial.print("X: ");
    Serial.print(event.magnetic.x);
    Serial.print(" Y: ");
    Serial.print(event.magnetic.y);
    Serial.print(" Z: ");
    Serial.println(event.magnetic.z);
  }
static float previousHeading = -1;  // начальное значение

if (millis() - lastHeadingTime >= 1000) {
  lastHeadingTime = millis();

  sensors_event_t event;
  compass.getEvent(&event);

  float heading = atan2(event.magnetic.y, event.magnetic.x);
  if (heading < 0)
    heading += 2 * PI;

  float headingDegrees = heading * 180 / PI;

  // Сравниваем с предыдущим значением (3% порог)
  if (previousHeading < 0 || abs(headingDegrees - previousHeading) / previousHeading > 0.03) {
    Serial.print("Heading: ");
    Serial.print(headingDegrees);
    Serial.println("°");

    previousHeading = headingDegrees;
  }
}



  delay(50);  // Умеренная задержка для обновления GPS/компаса
}
