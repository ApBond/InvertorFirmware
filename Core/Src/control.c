#include "control.h"

PIDHandle_t idLoopPID=
{
	.kp=0.045,
  .ki=0,
  .kd=0,
  .prevError=0,
  .integralTerm=0
};

PIDHandle_t iqLoopPID=
{
    .kp=0.045,
  .ki=0,
  .kd=0,
  .prevError=0,
  .integralTerm=0
};

extern float Ia;
extern float Ic;
extern float Ib;


float PIDController(PIDHandle_t * PID,float error)
{
    float deltaError = error - PID->prevError;
    float controllerOut;
    PID->prevError=error;
    PID->integralTerm+=error;
    controllerOut=error*PID->kp+PID->integralTerm*PID->ki+deltaError*PID->kd;
    return controllerOut;
}

void currentLoop(float Id,float Iq)
{
    d_q_t Us;
    d_q_t Is;
    alpha_betta_t I_alpha_betta;
    float rotorElAngle;
    I_alpha_betta.alpha=Ia;
    I_alpha_betta.betta=(Ia+2*Ib)/SQRT_3;
    rotorElAngle=getAngle();
    Is.d=I_alpha_betta.alpha*cosf(rotorElAngle)+I_alpha_betta.betta*sinf(rotorElAngle);
    Is.q=I_alpha_betta.betta*cosf(rotorElAngle)-I_alpha_betta.alpha*sinf(rotorElAngle);
    Us.d = PIDController(&idLoopPID,0-Is.d);
    Us.q = PIDController(&iqLoopPID,REF_TORQUE-Is.q);
    //if(Us.d>0.7) Us.d=0.7;
    //if(Us.q>0.7) Us.q=0.7;
    SVPWM_realise_dq(Us.d,Us.q,rotorElAngle,12);
}