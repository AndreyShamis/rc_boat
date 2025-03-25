#pragma once

#include <Arduino.h>

#define PPM_PIN 23

extern volatile uint16_t channels[10];
extern volatile uint32_t ppmCounter;

void setupFlySkyPPM();
uint16_t getFlySkyChannel(uint8_t channelIndex);
