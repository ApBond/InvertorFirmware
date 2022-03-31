#ifndef __HALL_FEEDBACK_H
#define __HALL_FEEDBACK_H

#include "main.h"

#define HALL_FILTER 0X7
#define HALL_MEASURE_COUNT 5
#define POLE_PAIRS 12

#define MIN_SPEED_RPM 5

#define d_ANGLE 1
#define d_ANGLE_RAD d_ANGLE*0.017453

#define OFFSET_ANGLE_DEG 0
#define OFFSET_ANGLE_RAD OFFSET_ANGLE_DEG*0.017453

#define HALL_ERROR_COUNT 1

#define SQRT_3 1.7320508
#define PI 3.1415926
#define PI_3 1.04719755
#define PI_2_3 2.0943951
#define PI_4_3 4.1887902
#define PI_5_3 5.23598776
#define PI_2 6.2831853
#define DEG_60 1.0472


static float hallGetAngle(uint8_t ha,uint8_t hb,uint8_t hc);
void hallInitAngle(uint8_t ha,uint8_t hb,uint8_t hc);
void hallTimInit(void);
float getSpeed(void);
float getAngle(void);
float getSinElAngle(void);
float getCosElAngle(void);

#endif