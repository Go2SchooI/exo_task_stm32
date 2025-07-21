#include "SSM_kinematics.h"
#include "exo_controller.h"

void SSM_inner_kinematics(float *theta, float alpha)
{
    exo_controller.xzy_shoulder.SSM.SSM_xzy_angle[0] = (theta[0] + theta[1]) / 2;
    exo_controller.xzy_shoulder.SSM.SSM_xzy_angle[1] = 6 * atanf(tanf(alpha) * cosf(theta[1] / 2 - theta[0] / 2));
    exo_controller.xzy_shoulder.SSM.SSM_xzy_angle[2] = theta[2];
}

void SSM_inner_inv_kinematics(float *xzy_angle, float alpha)
{
    exo_controller.xzy_shoulder.SSM.theta[0] = xzy_angle[0] - acosf(tanf(xzy_angle[1] / 6) / tanf(alpha));
    exo_controller.xzy_shoulder.SSM.theta[1] = xzy_angle[0] + acosf(tanf(xzy_angle[1] / 6) / tanf(alpha));
    exo_controller.xzy_shoulder.SSM.theta[2] = xzy_angle[2];
}

void SSM_inner_2_shoulder_angle(float *theta)
{
}