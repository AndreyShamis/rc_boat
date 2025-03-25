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

static uint32_t lastPpmCounter = 0;
static uint32_t lastPpmCheckTime = 0;

bool ppmSignalAlive = true;

// ==== Setup ====
void setup() {
  Serial.begin(115200);
  Serial.println("");
  delay(10);
  Serial.println("");
  delay(10);
  Serial.println("");
  delay(10);
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
  setRudderAngles(45, -45);
  delay(1000);
  setRudderAngles(-45, 45);
  delay(1000);
  setRudderAngle(0);
  setRudderAngles(-0, 0);

  setupFlySkyPPM();
  delay(10);
}

void setMotorPower(int percent) {
  setMotor(percent, percent);  // оба мотора с одинаковой мощностью
}


// ==== Loop ====
void loop() {
  updateEncoder();


  if (millis() - lastPpmCheckTime >= 500) {
    lastPpmCheckTime = millis();

    if (ppmCounter == lastPpmCounter) {
      ppmSignalAlive = false;  // Нет новых сигналов
    } else {
      ppmSignalAlive = true;  // Сигналы идут
      lastPpmCounter = ppmCounter;
    }
  }
  //Serial.printf("PPM Alive: %s (Counter: %lu)\n", ppmSignalAlive ? "YES" : "NO", ppmCounter);

  if (ppmSignalAlive) {
    uint16_t ch1 = getFlySkyChannel(0);
    uint16_t ch3 = getFlySkyChannel(2);
    //uint16_t ch6 = getFlySkyChannel(5);     // CH6 = индекс 5
    if (ch3> 999) {

      int rudder = map(ch1, 1000, 2000, -60, 60);
      int throttle = map(ch3, 1000, 2000, 0, 100);

      setRudderAngle(rudder);
      setMotorPower(throttle);
      //Serial.println("🛑 Failsafe active — transmitter disconnected.");
    }
  } else {
    // Serial.println("🚫 PPM signal lost — STOPPING");
    // setRudderAngle(0);
    // setMotorPower(0);
  }
  // static uint32_t lastCounter = 0;
  // if (ppmCounter != lastCounter) {
  //   Serial.print("PPM ISR Triggered: ");
  //   Serial.println(ppmCounter);
  //   lastCounter = ppmCounter;
  // }

  delay(1);
  static unsigned long lastStatusTime = 0;
  static unsigned long lastHeadingTime = 0;

  // ==== Read GPS ====
  while (GPSserial.available()) {
    char c = GPSserial.read();
    if (millis() - lastStatusTime >= 15000) {
      Serial.write(c);  //  NMEA
    }
    gps.encode(c);
  }

  // Каждые 15 секунд
  if (millis() - lastStatusTime >= 15000) {
    lastStatusTime = millis();

    if (gps.location.isValid()) {
      Serial.printf("✅ GPS FIX: Lat: %.6f, Lon: %.6f\n", gps.location.lat(), gps.location.lng());

    } else {
      Serial.println("🔄 Waiting for GPS fix...");
      if (gps.satellites.isValid()) {
        Serial.printf("Satellites in view: %d\n", gps.satellites.value());
      } else {
        Serial.printf("Satellites info not yet available.\n");
      }
      if (gps.altitude.isValid()) {
        Serial.printf("Altitude: %d\n", gps.altitude.meters());
      }
      if (gps.speed.isValid()) {
        Serial.printf("Speed: %d  km/h\n", gps.speed.kmph());
      }
    }
    // ==== Read Compass ====
    sensors_event_t event;
    compass.getEvent(&event);

    Serial.printf("X: %.2f Y: %.2f Z: %.2f\n", event.magnetic.x, event.magnetic.y, event.magnetic.z);
  }
  static float previousHeading = -1;

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
      Serial.printf("[DBG] Heading: %.2f°, X: %.2f, Y: %.2f, Z: %.2f, Lat: %.6f, Lon: %.6f\n",
                    headingDegrees,
                    event.magnetic.x, event.magnetic.y, event.magnetic.z,
                    gps.location.lat(), gps.location.lng());

      previousHeading = headingDegrees;
    }
  }
  delay(1);
}
