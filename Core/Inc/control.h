#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "math.h"

#define SPEED_LOOP_TIM_ARR 5


#define MAX_SPEED_RPM 500
#define MAX_TORQUE_AMP 10


typedef struct
{
    float kp;
    float ki;
    float kd;
    float prevError;
    float integralTerm;
}PIDHandle_t;

typedef enum
{
    SPEED_PID,
    ID_PID,
    IQ_PID
}Controller_type_t;

typedef enum
{
    SPEED_CONTROL,
    TORQUE_CONTROL
}FOC_Mode_t;

typedef enum
{
    NOT_ERROR,
    HALL_ERROR,
    SPEED_ERROR,
    OVERCURRENT_ERROR
}FOC_Error_t;

struct d_q_t1
{
    float d;
    float q;
};

typedef enum
{
    MS_RUN,
    MS_STOP
}Motor_state_t;

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
float getReferenceSpeed(void);
struct d_q_t getControllImpact(void);
void motorStart(void);
void motorStop(void);
void setControlMode(FOC_Mode_t mode);
void setPIDSettings(Controller_type_t type,PIDHandle_t controller);
void setErrorState(FOC_Error_t error);
FOC_Error_t getErrorState(void);
Motor_state_t getMotorState(void);

#endif