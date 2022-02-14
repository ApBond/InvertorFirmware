#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "currentSensing.h"

#define SPEED_LOOP_TIM_ARR 10

#define REF_TORQUE 3

typedef struct
{
    float kp;
    float ki;
    float kd;
    float prevError;
    float integralTerm;
}PIDHandle_t;

float PIDController(PIDHandle_t * PID,float error);
void currentLoop(float Id,float Iq,float rotorElAngle);
void regulatorClear(void);
void speedLoopTimerInit(void);
void startSpeedLoop(void);
void stopSpeedLoop(void);


#endif