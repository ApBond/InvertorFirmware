#ifndef __USER_COMMUNICATION
#define __USER_COMMUNICATION

#include "main.h"

#define BROADCAST_ID 1
#define SET_VECTOR_ID 2
#define INDIVIDUAL_ID1 101
#define INDIVIDUAL_ID2 INDIVIDUAL_ID1*10
#define ERROR_CHANNEL_ID 3

#define ALL_RESET 0x00
#define START_STOP_DRIVE 0x01
#define CHANGE_MODE 0x02
#define CHANGE_SPEED 0x03
#define CHANGE_TORQUE 0x04
#define CHANGE_ANGLE 0x05

#define MOTOR_START 0x01
#define MOTOR_STOP 0x00

#define TORQUE_MODE 0x00
#define SPEED_MODE 0x01

typedef enum{
    STOP,
    CALIBRATION,
    RUN
}Motion_mode_t;

void sendDiagnosticData(void);
void userCommunicationProcess(void);
void sendErrorState(void);

#endif