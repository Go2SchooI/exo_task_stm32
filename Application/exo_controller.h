#ifndef __EXO_CONTROLLER_H
#define __EXO_CONTROLLER_H

#include "includes.h"
#include "dm_imu.h"
#include "SSM_kinematics.h"
#include "exo_dynamics.h"

#define ELBOW_MOTOR_MIN -25.0f
#define ELBOW_MOTOR_MAX 135.0f

typedef struct
{
    float q[4], q_dot[4], q_ddot[4]; // 关节空间的位移、速度、加速度
} ctrl_human_value_t;

typedef struct
{
    dm_imu_t dm_imu;
    INS_t INS_shoulder;
    SSM_t SSM;

    float shoulder_ctrl_xzy_angle[3];
    float SSM_shoulder_xzy_angle[3]; // Human xzy angle from SSM,in degree

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

    INS_t INS_torso;
    EulerAnglesXZY shoulder_xzy_angle_wrt_torso;

    ExoDynamicsParams dynamics_params;
    float feedforward_output[4];
    ctrl_human_value_t ctrl_human_value;

    uint8_t motor_angle_check_flag;
} exo_controller_t;

enum
{
    SILENCE_MODE = 0,
    ANGLE_MODE,
    NORMAL_MODE,
    Velocity,
    Debug
};

void exo_init(void);
void exo_task(void);

extern exo_controller_t exo_controller;
extern uint8_t Mode;
extern float TargetVelocity, TargetAngle1;

#endif
