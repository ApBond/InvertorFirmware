#ifndef __USER_COMMUNICATION
#define __USER_COMMUNICATION

#include "main.h"
#include "servoControl.h"

#define BROADCAST_ID 1
#define SET_VECTOR_ID 2
#define ERROR_CHANNEL_ID 3
#define ROS_CONTROL_ID 4

#define ALL_RESET 0x00
#define START_STOP_DRIVE 0x01
#define CHANGE_MODE 0x02
#define CHANGE_SPEED 0x03
#define CHANGE_TORQUE 0x04
#define CHANGE_ANGLE 0x05
#define CHANGE_DEVICE_COMMUNICATION_MODE 0x06
#define CHANGE_MOTION_CONTROL_MODE 0x07
#define CHANGE_TO_BOOT_MODE 0xEE


#define MOTOR_START 0x01
#define MOTOR_STOP 0x00

#define TORQUE_MODE 0x00
#define SPEED_MODE 0x01

#define NO_SEND_DATA 0x00
#define SEND_DEBUG_DATA 0x01
#define SEND_ODOM_DATA 0x02

//Boot params
#define PROGRAMM_START_ADR 0x08002400
#define BOOT_KEY 0x861D8B36
#define BOOT_KEY_ADR 0x0800FFF0

typedef enum
{
    NO_DATA,
    DEBUG,
    ODOM
}Communication_mode_t;

typedef enum
{
    PDU_CONTROL,
    ROS_CONTROL
}Motion_control_mode_t;

typedef enum{
    STOP,
    CALIBRATION,
    RUN
}Motion_mode_t;

void sendDiagnosticData(void);
void userCommunicationProcess(void);
void sendErrorState(void);

#endif