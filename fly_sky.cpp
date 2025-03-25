#include "fly_sky.h"

volatile uint16_t channels[10];

volatile uint32_t ppmCounter = 0;


void IRAM_ATTR ppmInterrupt() {
  ppmCounter++;
  static uint32_t lastTime = 0;
  static uint8_t channelIndex = 0;

  uint32_t now = micros();
  uint32_t duration = now - lastTime;
  lastTime = now;

  if (duration > 3000) {
    channelIndex = 0;
  } else if (channelIndex < 10) {
    channels[channelIndex++] = duration;
  }
}

void setupFlySkyPPM() {
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterrupt, RISING);
}

uint16_t getFlySkyChannel(uint8_t channelIndex) {
  if (channelIndex >= 10) return 1500;
  return constrain(channels[channelIndex], 0, 2000);
}
