#ifndef __CURRENT_SENSING_H
#define __CURRENT_SENSING_H

#include "main.h"

#define ADC_CALIBRATION_COUNT 10
#define ADC_TO_AMP 0.00806
#define CURRENT_MEASURE_COUNT 2

#define CURRENT_LIM 15

typedef enum
{
 ADC_OK,
 ADC_NOT_INIT,
 ADC_CALIBRATION
}ADC_state_t;

typedef struct 
{
    float alpha;
    float betta;
}alpha_betta_t;

typedef struct 
{
    float d;
    float q;
}d_q_t;



ADC_state_t ADCInit(void);

#endif