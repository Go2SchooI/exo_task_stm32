#include "exo_dynamics.h"

// --- 3D向量与矩阵运算辅助函数 ---
static void vector_add(const float v_a[3], const float v_b[3], float v_out[3]);
static void vector_sub(const float v_a[3], const float v_b[3], float v_out[3]);
static void vector_cross(const float v_a[3], const float v_b[3], float v_out[3]);
static void vector_scale(const float v_in[3], float s, float v_out[3]);
static float vector_dot(const float v_a[3], const float v_b[3]);
static void matrix_vec_mult(const mat *M, const float v_in[3], float v_out[3]);
static void rotation_from_axis_angle(const float axis[3], float angle, mat *pR, float *pR_Data);

// v_out = v_a + v_b
static void vector_add(const float v_a[3], const float v_b[3], float v_out[3])
{
    v_out[0] = v_a[0] + v_b[0];
    v_out[1] = v_a[1] + v_b[1];
    v_out[2] = v_a[2] + v_b[2];
}

// v_out = v_a - v_b
static void vector_sub(const float v_a[3], const float v_b[3], float v_out[3])
{
    v_out[0] = v_a[0] - v_b[0];
    v_out[1] = v_a[1] - v_b[1];
    v_out[2] = v_a[2] - v_b[2];
}

// v_out = v_a x v_b (叉乘)
static void vector_cross(const float v_a[3], const float v_b[3], float v_out[3])
{
    v_out[0] = v_a[1] * v_b[2] - v_a[2] * v_b[1];
    v_out[1] = v_a[2] * v_b[0] - v_a[0] * v_b[2];
    v_out[2] = v_a[0] * v_b[1] - v_a[1] * v_b[0];
}

// v_out = s * v_in (标量乘法)
static void vector_scale(const float v_in[3], float s, float v_out[3])
{
    v_out[0] = v_in[0] * s;
    v_out[1] = v_in[1] * s;
    v_out[2] = v_in[2] * s;
}

// dot_product = v_a · v_b (点乘)
static float vector_dot(const float v_a[3], const float v_b[3])
{
    return v_a[0] * v_b[0] + v_a[1] * v_b[1] + v_a[2] * v_b[2];
}

// v_out = M * v_in (3x3矩阵乘以3x1向量)
static void matrix_vec_mult(const mat *M, const float v_in[3], float v_out[3])
{
    v_out[0] = M->pData[0] * v_in[0] + M->pData[1] * v_in[1] + M->pData[2] * v_in[2];
    v_out[1] = M->pData[3] * v_in[0] + M->pData[4] * v_in[1] + M->pData[5] * v_in[2];
    v_out[2] = M->pData[6] * v_in[0] + M->pData[7] * v_in[1] + M->pData[8] * v_in[2];
}

// 根据轴和角度生成旋转矩阵
static void rotation_from_axis_angle(const float axis[3], float angle, mat *pR, float *pR_Data)
{
    float c = arm_cos_f32(angle);
    float s = arm_sin_f32(angle);
    float t = 1.0f - c;
    float x = axis[0], y = axis[1], z = axis[2];
    pR_Data[0] = t * x * x + c;
    pR_Data[1] = t * x * y - s * z;
    pR_Data[2] = t * x * z + s * y;
    pR_Data[3] = t * x * y + s * z;
    pR_Data[4] = t * y * y + c;
    pR_Data[5] = t * y * z - s * x;
    pR_Data[6] = t * x * z - s * y;
    pR_Data[7] = t * y * z + s * x;
    pR_Data[8] = t * z * z + c;
    Matrix_Init(pR, 3, 3, pR_Data);
}

void exo_dynamics_params_init(ExoDynamicsParams *params)
{
    // 上臂
    params->m1 = 0.352f + 1.46f;
    params->p12[0] = 0;
    params->p12[1] = 0;
    params->p12[2] = -0.3f; // 肩到肘
    params->pc1[0] = 0;
    params->pc1[1] = 0;
    params->pc1[2] = -127.072f / 1000;                        // 肩到上臂质心
    float I1_val = (1.0f / 12.0f) * params->m1 * 0.3f * 0.3f; // 细杆近似, 0.3为细杆长度
    params->I1_data[0] = 0.001f;                              // 使用一个极小的非零正数（比如 0.001）来代替0
    params->I1_data[1] = 0;
    params->I1_data[2] = 0; // 绕自身轴的转动惯量
    params->I1_data[3] = 0;
    params->I1_data[4] = I1_val;
    params->I1_data[5] = 0;
    params->I1_data[6] = 0;
    params->I1_data[7] = 0;
    params->I1_data[8] = I1_val;
    Matrix_Init(&params->I1, 3, 3, params->I1_data);

    // 前臂
    params->m2 = 0.16f + 0.75f;
    params->pc2[0] = 0;
    params->pc2[1] = 0;
    params->pc2[2] = -59.216f / 1000;                           // 肘到前臂质心
    float I2_val = (1.0f / 12.0f) * params->m2 * 0.25f * 0.25f; // 细杆近似
    params->I2_data[0] = 0.001f;
    params->I2_data[1] = 0;
    params->I2_data[2] = 0;
    params->I2_data[3] = 0;
    params->I2_data[4] = I2_val;
    params->I2_data[5] = 0;
    params->I2_data[6] = 0;
    params->I2_data[7] = 0;
    params->I2_data[8] = I2_val;
    Matrix_Init(&params->I2, 3, 3, params->I2_data);
    // 重力
    params->g[0] = 0;
    params->g[1] = 0;
    params->g[2] = -9.81f;
}

// =================================================================================
// Section 2: 递归牛顿-欧拉算法 (RNEA)
// =================================================================================

/**
 * @brief 使用RNEA计算动力学前馈力矩
 * @param params 指向包含所有物理参数的结构体
 * @param q 4x1 关节角度数组 [q_s1, q_s2, q_s3, q_e1] (rad)
 * @param q_dot 4x1 关节角速度数组 (rad/s)
 * @param q_ddot 4x1 关节角加速度数组 (rad/s^2)
 * @param tau_ff [输出] 4x1 数组，用于存储计算出的前馈力矩 (N·m)
 */
void calculate_dynamics_feedforward(ExoDynamicsParams *params, const float q[4], const float q_dot[4], const float q_ddot[4], float *tau_ff)
{
    // --- 关节轴定义 (在各自的连杆坐标系中) ---
    // 假设肩关节为 X-Z-Y 欧拉角, 肘关节为 Y 轴
    const float x_axis[3] = {1, 0, 0};
    const float y_axis[3] = {0, 1, 0};
    const float z_axis[3] = {0, 0, 1};

    // --- 计算旋转矩阵 ---
    mat R01, R12, R23, R34; // 相对旋转
    float R01_data[9], R12_data[9], R23_data[9], R34_data[9];
    rotation_from_axis_angle(x_axis, q[0], &R01, R01_data); // 绕 X 轴
    rotation_from_axis_angle(z_axis, q[1], &R12, R12_data); // 绕 Z 轴
    rotation_from_axis_angle(y_axis, q[2], &R23, R23_data); // 绕 Y 轴
    rotation_from_axis_angle(y_axis, q[3], &R34, R34_data); // 肘关节绕 Y 轴

    mat R02, R03, R04; // 绝对旋转 (相对于基座)
    float R02_data[9], R03_data[9], R04_data[9];
    Matrix_Init(&R02, 3, 3, R02_data);
    Matrix_Init(&R03, 3, 3, R03_data);
    Matrix_Init(&R04, 3, 3, R04_data);
    Matrix_Multiply(&R01, &R12, &R02);
    Matrix_Multiply(&R02, &R23, &R03); // R03: 上臂姿态
    Matrix_Multiply(&R03, &R34, &R04); // R04: 前臂姿态

    // --- RNEA: 正向传播 (速度和加速度) ---
    float w1[3] = {0}, w2[3] = {0};           // 连杆角速度
    float w_dot1[3] = {0}, w_dot2[3] = {0};   // 连杆角加速度
    float v_dot1[3] = {0}, v_dot2[3] = {0};   // 关节线加速度
    float vc_dot1[3] = {0}, vc_dot2[3] = {0}; // 质心线加速度

    // 临时变量
    float temp3x1_a[3], temp3x1_b[3];

    // -- 连杆1 (上臂) --
    // w1 = (q_dot0*x) + R01*(q_dot1*z) + R02*(q_dot2*y)
    vector_scale(x_axis, q_dot[0], w1);
    vector_scale(z_axis, q_dot[1], temp3x1_a);
    matrix_vec_mult(&R01, temp3x1_a, temp3x1_b);
    vector_add(w1, temp3x1_b, w1);
    vector_scale(y_axis, q_dot[2], temp3x1_a);
    matrix_vec_mult(&R02, temp3x1_a, temp3x1_b);
    vector_add(w1, temp3x1_b, w1);

    // w_dot1 = (q_ddot0*x) + R01*(q_ddot1*z) + R02*(q_ddot2*y) + ... (科里奥利项)
    vector_scale(x_axis, q_ddot[0], w_dot1);
    vector_scale(z_axis, q_ddot[1], temp3x1_a);
    matrix_vec_mult(&R01, temp3x1_a, temp3x1_b);
    vector_add(w_dot1, temp3x1_b, w_dot1);
    vector_scale(y_axis, q_ddot[2], temp3x1_a);
    matrix_vec_mult(&R02, temp3x1_a, temp3x1_b);
    vector_add(w_dot1, temp3x1_b, w_dot1);
    // 加上科里奥利加速度项
    vector_scale(z_axis, q_dot[1], temp3x1_a);
    matrix_vec_mult(&R01, temp3x1_a, temp3x1_b);
    vector_cross(w1, temp3x1_b, temp3x1_a);
    vector_add(w_dot1, temp3x1_a, w_dot1);
    vector_scale(y_axis, q_dot[2], temp3x1_a);
    matrix_vec_mult(&R02, temp3x1_a, temp3x1_b);
    vector_cross(w1, temp3x1_b, temp3x1_a);
    vector_add(w_dot1, temp3x1_a, w_dot1);

    // vc_dot1 = w_dot1 x pc1 + w1 x (w1 x pc1) - g
    vector_cross(w1, params->pc1, temp3x1_a);
    vector_cross(w1, temp3x1_a, temp3x1_b);
    vector_cross(w_dot1, params->pc1, temp3x1_a);
    vector_add(temp3x1_a, temp3x1_b, vc_dot1);
    vector_sub(vc_dot1, params->g, vc_dot1);

    // v_dot1 (肘关节的线加速度)
    vector_cross(w1, params->p12, temp3x1_a);
    vector_cross(w1, temp3x1_a, temp3x1_b);
    vector_cross(w_dot1, params->p12, temp3x1_a);
    vector_add(temp3x1_a, temp3x1_b, v_dot1);
    vector_sub(v_dot1, params->g, v_dot1);

    // -- 连杆2 (前臂) --
    // w2 = w1 + R03*(q_dot3*y)
    vector_scale(y_axis, q_dot[3], temp3x1_a);
    matrix_vec_mult(&R03, temp3x1_a, temp3x1_b);
    vector_add(w1, temp3x1_b, w2);

    // w_dot2 = w_dot1 + R03*(q_ddot3*y) + w1 x (R03*q_dot3*y)
    vector_scale(y_axis, q_ddot[3], temp3x1_a);
    matrix_vec_mult(&R03, temp3x1_a, temp3x1_b);
    vector_add(w_dot1, temp3x1_b, w_dot2);
    vector_scale(y_axis, q_dot[3], temp3x1_a);
    matrix_vec_mult(&R03, temp3x1_a, temp3x1_b);
    vector_cross(w1, temp3x1_b, temp3x1_a);
    vector_add(w_dot2, temp3x1_a, w_dot2);

    // vc_dot2 = v_dot1 + w_dot2 x pc2 + w2 x (w2 x pc2)
    vector_cross(w2, params->pc2, temp3x1_a);
    vector_cross(w2, temp3x1_a, temp3x1_b);
    vector_cross(w_dot2, params->pc2, temp3x1_a);
    vector_add(temp3x1_a, temp3x1_b, vc_dot2);
    vector_add(vc_dot2, v_dot1, vc_dot2);

    // --- RNEA: 反向传播 (力和力矩) ---
    float f1[3], f2[3]; // 连杆受力
    float n1[3], n2[3]; // 连杆所受力矩

    // -- 连杆2 (前臂) --
    // F2 = m2 * vc_dot2
    vector_scale(vc_dot2, params->m2, f2);
    // N2 = I2*w_dot2 + w2 x (I2*w2)
    matrix_vec_mult(&params->I2, w2, temp3x1_a);
    vector_cross(w2, temp3x1_a, temp3x1_b);
    matrix_vec_mult(&params->I2, w_dot2, temp3x1_a);
    vector_add(temp3x1_a, temp3x1_b, n2);

    // -- 连杆1 (上臂) --
    // f1 = f2 + m1*vc_dot1
    vector_scale(vc_dot1, params->m1, temp3x1_a);
    vector_add(f2, temp3x1_a, f1);
    // n1 = n2 + (p12 x f2) + (pc1 x m1*vc_dot1) + I1*w_dot1 + w1 x (I1*w1)
    matrix_vec_mult(&params->I1, w1, temp3x1_a);
    vector_cross(w1, temp3x1_a, temp3x1_b);
    matrix_vec_mult(&params->I1, w_dot1, temp3x1_a);
    vector_add(temp3x1_a, temp3x1_b, n1);
    vector_cross(params->p12, f2, temp3x1_a);
    vector_add(n1, temp3x1_a, n1);
    vector_scale(vc_dot1, params->m1, temp3x1_b);
    vector_cross(params->pc1, temp3x1_b, temp3x1_a);
    vector_add(n1, temp3x1_a, n1);
    vector_add(n1, n2, n1); // 加上从连杆2传递过来的力矩

    // --- 计算关节力矩 ---
    float axis_s1[3], axis_s2[3], axis_s3[3], axis_e1[3];
    // 关节1 轴 (X) 在世界系中
    vector_scale(x_axis, 1.0f, axis_s1);
    // 关节 2 轴 (Z) 在世界系中
    matrix_vec_mult(&R01, z_axis, axis_s2);
    // 关节 3 轴 (Y) 在世界系中
    matrix_vec_mult(&R02, y_axis, axis_s3);
    // 关节 4 轴 (Y) 在世界系中
    matrix_vec_mult(&R03, y_axis, axis_e1);

    tau_ff[0] = vector_dot(n1, axis_s1);
    tau_ff[1] = vector_dot(n1, axis_s2);
    tau_ff[2] = vector_dot(n1, axis_s3);
    tau_ff[3] = vector_dot(n2, axis_e1);
}

// =================================================================================
// Section 3: 主函数和示例
// =================================================================================

// int main()
// {
//     printf("--- 动力学前馈计算示例 (X-Z-Y 肩关节) ---\n");

//     // 1. 初始化物理参数 (这些值需要根据你的实际外骨骼测量或估算)
//     ExoDynamicsParams exo_params;

//     // 2. 设置期望的运动状态 (轨迹规划器的输出)
//     // 状态1: 悬停，有重力，无速度和加速度
//     printf("\n状态1: 悬停 (仅重力补偿)\n");
//     float q1[4] = {PI / 4.f, PI / 6.f, 0.f, PI / 3.f}; // 期望角度 (rad)
//     float q_dot1[4] = {0.f, 0.f, 0.f, 0.f};            // 期望速度 (rad/s)
//     float q_ddot1[4] = {0.f, 0.f, 0.f, 0.f};           // 期望加速度 (rad/s^2)
//     float tau_ff1[4];
//     calculate_dynamics_feedforward(&exo_params, q1, q_dot1, q_ddot1, tau_ff1);
//     printf("期望角度q=[%.2f, %.2f, %.2f, %.2f] rad\n", q1[0], q1[1], q1[2], q1[3]);
//     printf("Tau=[%.3f, %.3f, %.3f, %.3f] Nm\n", tau_ff1[0], tau_ff1[1], tau_ff1[2], tau_ff1[3]);

//     // 状态2: 运动中，包含所有项
//     printf("\n状态2: 运动中 (完整动力学)\n");
//     float q2[4] = {0.1f, 0.2f, 0.3f, 0.5f};      // 期望角度 (rad)
//     float q_dot2[4] = {0.5f, 0.4f, 0.2f, 0.6f};  // 期望速度 (rad/s)
//     float q_ddot2[4] = {1.0f, 0.5f, 0.8f, 1.2f}; // 期望加速度 (rad/s^2)
//     float tau_ff2[4];
//     calculate_dynamics_feedforward(&exo_params, q2, q_dot2, q_ddot2, tau_ff2);
//     printf("期望角度q=[%.2f, %.2f, %.2f, %.2f] rad\n", q2[0], q2[1], q2[2], q2[3]);
//     printf("期望速度q_dot=[%.2f, %.2f, %.2f, %.2f] rad/s\n", q_dot2[0], q_dot2[1], q_dot2[2], q_dot2[3]);
//     printf("Tau=[%.3f, %.3f, %.3f, %.3f] Nm\n", tau_ff2[0], tau_ff2[1], tau_ff2[2], tau_ff2[3]);

//     return 0;
// }
