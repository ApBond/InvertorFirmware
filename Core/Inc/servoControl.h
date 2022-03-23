#ifndef _SERVOCONTROL_H
#define _SERVOCONTROL_H

#include "main.h"

#define REDUCTOR 1 //30
#define RESOLUTION 200

uint8_t calibration();
void tim16Init(void);
void getServoAngle();

#endif

