#pragma once

extern int currentLeft;
extern int currentRight;

void setMotor(int leftPercent, int rightPercent);
void setMotorSpeeds(int leftThrottle, int rightThrottle);
int mapWithDeadzone(int percent);
