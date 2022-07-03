#ifndef _SERVOCONTROL_H
#define _SERVOCONTROL_H

#include "main.h"

#define REDUCTOR 20 //30
#define RESOLUTION 800

uint8_t calibration();
void tim16Init(void);
void getServoAngle();
float odomServoGetAngle(void);

#endif

