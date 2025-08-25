#ifndef __EXO_DYNAMICS_H
#define __EXO_DYNAMICS_H

#include "includes.h"

// --- 用户提供的矩阵运算库宏定义 ---
#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Multiply arm_mat_mult_f32

// =================================================================================
// Section 1: 数据结构与辅助函数
// =================================================================================

/**
 * @brief 定义外骨骼连杆的物理参数
 */
typedef struct
{
    // --- 上臂 (Link 1) ---
    float m1; // 质量 (kg)
    mat I1;   // 3x3 惯性张量 (绕质心)
    float I1_data[9];
    float pc1[3]; // 从关节1(肩)到质心1的向量 (m)

    // --- 前臂 (Link 2) ---
    float m2; // 质量 (kg)
    mat I2;   // 3x3 惯性张量 (绕质心)
    float I2_data[9];
    float pc2[3]; // 从关节2(肘)到质心2的向量 (m)

    // --- 连杆几何 ---
    float p12[3]; // 从关节1(肩)到关节2(肘)的向量 (m)

    // --- 环境 ---
    float g[3]; // 重力加速度向量 (m/s^2)

} ExoDynamicsParams;

void exo_dynamics_params_init(ExoDynamicsParams *params);
void calculate_dynamics_feedforward(ExoDynamicsParams *params, const float q[4],
                                    const float q_dot[4], const float q_ddot[4], float *tau_ff);

#endif