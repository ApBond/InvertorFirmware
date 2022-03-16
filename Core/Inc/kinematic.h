#ifndef _KINEMATIC_H
#define _KINEMATIC_H

#include "main.h"

typedef struct 
{
    float V;
    float R;
    float gam;
} rcCommand_t;

typedef struct 
{
    uint8_t wheel;
    float V;
    float gam;
} Odom_t;

typedef struct 
{
    float MotorSpeed;
    float w;
    float gam;
} driveControl_t;

// #define FRONT_LEFT
#define FRONT_RIGHT
// #define REAR_LEFT
// #define REAR_RIGHT

#define pi 3.14159265
#define deg 57.29577951
#define CONSTRAIN 5
#define L 0.945
#define C 0.63
#define r_wheel 0.2

#define FL 0
#define FR 1
#define RL 2
#define RR 3



#ifdef FRONT_LEFT
    #define WHEEL FL
#endif
#ifdef FRONT_RIGHT
    #define WHEEL FR
#endif 
#ifdef REAR_LEFT
    #define WHEEL RL
#endif 
#ifdef REAR_RIGHT
    #define WHEEL RR
#endif

double sign(double a);
int16_t intSign(double a);
void kinematica(rcCommand_t cmd, float* Speed, float* targetAngle);

#endif