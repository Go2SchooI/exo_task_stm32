#ifndef __MOTOR_TASK_H
#define __MOTOR_TASK_H

#include "includes.h"

#define MOTOR_ZERO_OFFSET 45558

#define MOTOR1_MIN 107.066223
#define MOTOR1_MAX 149.883972

#define MOTOR2_MIN -41.036377
#define MOTOR2_MAX -0.338777393

typedef struct
{
    uint8_t mode, last_mode;

    uint32_t DWT_count;
    float mode_change_timestamp;
    float dt, t;

    uint32_t CAN_send_error_count;
    uint8_t CAN_send_status;
    motor_info lk_motor, dm_motor[3];
} exo_controller_t;

enum
{
    SILENCE_MODE = 0,
    Angle,
    Velocity,
    Debug
};

void exo_init(void);
void exo_task(void);

extern exo_controller_t exo_controller;
extern uint8_t Mode;
extern float TargetVelocity, TargetAngle1;

#endif
