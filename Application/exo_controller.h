#ifndef __EXO_CONTROLLER_H
#define __EXO_CONTROLLER_H

#include "includes.h"
#include "dm_imu.h"
#include "SSM_kinematics.h"

#define ELBOW_MOTOR_MIN -90.0f
#define ELBOW_MOTOR_MAX 90.0f

typedef struct
{
    dm_imu_t dm_imu;
    INS_t INS_shoulder;
    SSM_t SSM;

    motor_info lk_motor,
        dm_motor[2];
} exo_shoulder_t;

typedef struct
{
    motor_info dm_motor;
    float elbow_angle; // Elbow angle in degrees
} exo_elbow_t;

typedef struct
{
    uint8_t mode, last_mode;
    uint8_t debug_mode;

    uint32_t DWT_count;
    float mode_change_timestamp;
    float dt, t;

    uint32_t CAN_send_error_count;
    uint8_t CAN_send_status;

    exo_shoulder_t xzy_shoulder;
    exo_elbow_t elbow;
} exo_controller_t;

enum
{
    SILENCE_MODE = 0,
    ANGLE_MODE,
    Velocity,
    Debug
};

void exo_init(void);
void exo_task(void);

extern exo_controller_t exo_controller;
extern uint8_t Mode;
extern float TargetVelocity, TargetAngle1;

#endif
