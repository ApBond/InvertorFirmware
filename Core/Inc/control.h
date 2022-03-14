#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "currentSensing.h"
#include "math.h"

#define SPEED_LOOP_TIM_ARR 5


#define MAX_SPEED_RPM 200
#define MAX_TORQUE_AMP 15


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
void speedLoop(void);
void speedLoopTimerInit(void);
void startSpeedLoop(void);
void stopSpeedLoop(void);
void setReferenceSpeed(float speed);
void setReferenceTorque(float torque);
float getReferenceTorque(void);
//d_q_t circleLimitation(d_q_t vector);

#endif