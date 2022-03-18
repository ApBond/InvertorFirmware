#ifndef __CURRENT_SENSING_H
#define __CURRENT_SENSING_H

#include "main.h"

#define ADC_CALIBRATION_COUNT 10
#define ADC_TO_AMP 0.00806
#define CURRENT_MEASURE_COUNT 2

#define CURRENT_LIM 20


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
    float a;
    float b;
    float c;
}abc_t;

struct d_q_t 
{
    float d;
    float q;
};


ADC_state_t ADCInit(void);
abc_t getCurrentIabc(void);
struct d_q_t getCurrentIdq(void);

#endif