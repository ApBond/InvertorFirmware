#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "currentSensing.h"

#define REF_TORQUE 2

typedef struct
{
    float kp;
    float ki;
    float kd;
    float prevError;
    float integralTerm;
}PIDHandle_t;

float PIDController(PIDHandle_t * PID,float error);
void currentLoop(float Id,float Iq);

#endif