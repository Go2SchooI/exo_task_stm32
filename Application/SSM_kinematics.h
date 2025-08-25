#ifndef __SSM_KINEMATICS_H
#define __SSM_KINEMATICS_H

#include "includes.h"
#include "kalman_filter.h"

#define MAX_SSM_MOTOR_ANFLE_DIFFERENCE 75.0f // Maximum angle difference for SSM motors

#define SSM_MOTOR1_MIN -95.0f
#define SSM_MOTOR1_MAX 45.0f
#define SSM_MOTOR2_MIN -75.0f
#define SSM_MOTOR2_MAX 60.0f

#define SSM_Y_ANGLE_MIN -120.0f
#define SSM_Y_ANGLE_MAX 120.0f

typedef struct
{
    float alpha, offset_Z;
    float theta[3], phi[3];
    float SSM_xzy_angle[3];
} SSM_t;

void SSM_inner_forward_kinematics(float *theta, float alpha, float *phi);
void SSM_inner_inv_kinematics(float *xzy_angle, float alpha, float *theta);
void SSM_2_shoulder_angle(float *phi, float offsetZ, float *human_xzy_angle);
void shoulder_angle_2_SSM(float *human_xzy_angle, float offsetZ, float *ssm_xzy_angle);

#endif