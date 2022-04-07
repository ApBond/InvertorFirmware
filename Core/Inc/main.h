#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f303x8.h"
#include "rcc.h"
#include "can.h"
#include "hallFeedback.h"
#include "currentSensing.h"
#include "control.h"
#include "PWMControl.h"
#include "userCommunication.h"
#include "kinematic.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define COMMUNICATE_PERIOD 5

#define FRONT_LEFT
//#define FRONT_RIGHT
 //#define REAR_LEFT
// #define REAR_RIGHT

#ifdef FRONT_LEFT
    #define WHEEL FL
    #define INDIVIDUAL_ID1 101
#endif
#ifdef FRONT_RIGHT
    #define WHEEL FR
    #define INDIVIDUAL_ID1 102
#endif 
#ifdef REAR_LEFT
    #define WHEEL RL
    #define INDIVIDUAL_ID1 103
#endif 
#ifdef REAR_RIGHT
    #define WHEEL RR
    #define INDIVIDUAL_ID1 104
#endif

#define INDIVIDUAL_ID2 INDIVIDUAL_ID1*10

#endif