#ifndef __SSM_KINEMATICS_H
#define __SSM_KINEMATICS_H

#include "includes.h"
#include "kalman_filter.h"

#define MAX_SSM_MOTOR_ANFLE_DIFFERENCE 75.0f // Maximum angle difference for SSM motors

#define SSM_MOTOR1_MIN -95.0f //-95
#define SSM_MOTOR1_MAX 45.0f  // 45
#define SSM_MOTOR2_MIN -75.0f //-75
#define SSM_MOTOR2_MAX 60.0f  // 60

#define SSM_Y_ANGLE_MIN -120.0f //-120
#define SSM_Y_ANGLE_MAX 120.0f  // 120

typedef struct
{
    float alpha, offset_Z;  // rad
    float theta[3], phi[3]; // rad

    float SSM_ctrl_theta[3];
    float SSM_ctrl_xzy_angle[3];

    float offset_theta;
} SSM_t;

void SSM_inner_forward_kinematics(float *theta, float alpha, float *phi);
void SSM_inner_inv_kinematics(float *xzy_angle, float alpha, float *theta);
void SSM_2_shoulder_angle(float *phi, float offsetZ, float *shoulder_xzy_angle);
void shoulder_angle_2_SSM(float *shoulder_xzy_angle, float offsetZ, float *ssm_xzy_angle);

#endif