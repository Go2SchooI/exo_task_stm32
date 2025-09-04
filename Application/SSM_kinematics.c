#include "SSM_kinematics.h"

static void rotation_from_axis_angle(float axis[3], float angle, mat *pR, float *pR_Data);

void SSM_inner_forward_kinematics(float *theta, float alpha, float *phi)
{
    phi[0] = (theta[0] + theta[1]) / 2;
    phi[1] = 4 * atanf(tanf(alpha) * cosf(theta[1] / 2 - theta[0] / 2));
    phi[2] = theta[2];
}

void SSM_inner_inv_kinematics(float *xzy_angle, float alpha, float *theta)
{
    theta[0] = xzy_angle[0] - acosf(tanf(xzy_angle[1] / 4) / tanf(alpha));
    theta[1] = xzy_angle[0] + acosf(tanf(xzy_angle[1] / 4) / tanf(alpha));
    theta[2] = xzy_angle[2];
}

/**
 * @brief 根据旋转轴和角度生成旋转矩阵 (罗德里格斯公式)
 * @param axis 长度为3的数组，表示单位旋转轴 [x, y, z]
 * @param angle 旋转角度 (弧度)
 * @param pR 指向目标矩阵实例的指针，函数将在此初始化并填充数据
 * @param pR_Data 指向一个 3x3=9 个元素的浮点数组，用于存储矩阵数据
 */
static void rotation_from_axis_angle(float axis[3], float angle, mat *pR, float *pR_Data)
{
    float c = arm_cos_f32(angle);
    float s = arm_sin_f32(angle);
    float t = 1.0f - c;
    float x = axis[0];
    float y = axis[1];
    float z = axis[2];

    // 罗德里格斯公式展开，按行主序填充数据
    pR_Data[0] = t * x * x + c;
    pR_Data[1] = t * x * y - s * z;
    pR_Data[2] = t * x * z + s * y;
    pR_Data[3] = t * x * y + s * z;
    pR_Data[4] = t * y * y + c;
    pR_Data[5] = t * y * z - s * x;
    pR_Data[6] = t * x * z - s * y;
    pR_Data[7] = t * y * z + s * x;
    pR_Data[8] = t * z * z + c;

    // 初始化矩阵实例，将其与数据缓冲区关联
    Matrix_Init(pR, 3, 3, pR_Data);
}

void SSM_2_shoulder_angle(float *phi, float offsetZ, float *shoulder_xzy_angle)
{
    mat R_e1, R_e2, R_e3, R_e_temp, R_e_final;

    float R_e1_data[9], R_e2_data[9], R_e3_data[9];
    float R_e_temp_data[9], R_e_final_data[9];

    // 1. 定义外骨骼的旋转轴 (w_e 向量)
    float cos_oz = arm_cos_f32(offsetZ);
    float sin_oz = arm_sin_f32(offsetZ);

    float w_e1[3] = {cos_oz, sin_oz, 0.0f};
    float w_e2[3] = {0.0f, 0.0f, 1.0f};
    float w_e3[3] = {0.0f, 1.0f, 0.0f};

    // 2. 计算外骨骼每个关节对应的独立旋转矩阵
    rotation_from_axis_angle(w_e1, phi[0], &R_e1, R_e1_data);
    rotation_from_axis_angle(w_e2, phi[1], &R_e2, R_e2_data);
    rotation_from_axis_angle(w_e3, phi[2], &R_e3, R_e3_data);

    Matrix_Init(&R_e_temp, 3, 3, R_e_temp_data);
    Matrix_Init(&R_e_final, 3, 3, R_e_final_data);

    // R_e_temp = R_e1 * R_e2
    Matrix_Multiply(&R_e1, &R_e2, &R_e_temp);
    // R_e_final = R_e_temp * R_e3
    Matrix_Multiply(&R_e_temp, &R_e3, &R_e_final);

    // 矩阵元素通过 pData 指针访问: M(row,col) -> M.pData[row * numCols + col]
    // a2 = asin(-R(1,2)) -> -R_e_final.pData[0*3 + 1]
    float a1, a2, a3;
    float sin_a2 = -R_e_final.pData[1];
    if (sin_a2 > 1.0f)
        sin_a2 = 1.0f;
    if (sin_a2 < -1.0f)
        sin_a2 = -1.0f;
    a2 = asinf(sin_a2);

    // 检查是否处于奇异点（万向节锁）
    if (fabsf(arm_cos_f32(a2)) > 1e-6f)
    {
        // 非奇异情况
        // a3 = atan2(R(1,3), R(1,1))
        a3 = atan2f(R_e_final.pData[2], R_e_final.pData[0]);
        // a1 = atan2(R(3,2), R(2,2))
        a1 = atan2f(R_e_final.pData[7], R_e_final.pData[4]);
    }
    else
    {
        // 奇异情况
        a3 = 0.0f;
        a1 = atan2f(-R_e_final.pData[5], R_e_final.pData[8]);
    }

    shoulder_xzy_angle[0] = a1 * RADIAN_COEF;
    shoulder_xzy_angle[1] = a2 * RADIAN_COEF;
    shoulder_xzy_angle[2] = a3 * RADIAN_COEF;
}

void shoulder_angle_2_SSM(float *shoulder_xzy_angle, float offsetZ, float *ssm_xzy_angle)
{
    // --- 声明矩阵实例和数据缓冲区 ---
    mat Rz_neg20, Rx_q1, Rz_q2, Ry_q3;
    mat R_temp1, R_temp2, R_final;

    float Rz_neg20_data[9], Rx_q1_data[9], Rz_q2_data[9], Ry_q3_data[9];
    float R_temp1_data[9], R_temp2_data[9], R_final_data[9];

    // 1. 定义旋转轴和固定角度
    float z_angle_rad = -offsetZ; // 对应 rotz(20).'
    float axis_x[3] = {1.0f, 0.0f, 0.0f};
    float axis_y[3] = {0.0f, 1.0f, 0.0f};
    float axis_z[3] = {0.0f, 0.0f, 1.0f};

    // 2. 计算每个独立的旋转矩阵
    rotation_from_axis_angle(axis_z, z_angle_rad, &Rz_neg20, Rz_neg20_data);
    rotation_from_axis_angle(axis_x, shoulder_xzy_angle[0], &Rx_q1, Rx_q1_data);
    rotation_from_axis_angle(axis_z, shoulder_xzy_angle[1], &Rz_q2, Rz_q2_data);
    rotation_from_axis_angle(axis_y, shoulder_xzy_angle[2], &Ry_q3, Ry_q3_data);

    // 3. 按照顺序连乘: R_final = Rz(-20) * Rx(q1) * Rz(q2) * Ry(q3)
    Matrix_Init(&R_temp1, 3, 3, R_temp1_data);
    Matrix_Init(&R_temp2, 3, 3, R_temp2_data);
    Matrix_Init(&R_final, 3, 3, R_final_data);

    Matrix_Multiply(&Rz_neg20, &Rx_q1, &R_temp1);
    Matrix_Multiply(&R_temp1, &Rz_q2, &R_temp2);
    Matrix_Multiply(&R_temp2, &Ry_q3, &R_final);

    // 4. 从最终的姿态矩阵中反解出外骨骼关节角度 (y1, y2, y3)
    float y1, y2, y3;
    float sin_y2 = -R_final.pData[1]; // -R(1,2)
    if (sin_y2 > 1.0f)
        sin_y2 = 1.0f;
    if (sin_y2 < -1.0f)
        sin_y2 = -1.0f;
    y2 = asinf(sin_y2);

    if (fabsf(arm_cos_f32(y2)) > 1e-6f)
    {
        // 非奇异情况
        y3 = atan2f(R_final.pData[2], R_final.pData[0]); // atan2(R(1,3), R(1,1))
        y1 = atan2f(R_final.pData[7], R_final.pData[4]); // atan2(R(3,2), R(2,2))
    }
    else
    {
        // 奇异情况
        y3 = 0.0f;
        y1 = atan2f(-R_final.pData[5], R_final.pData[8]);
    }

    ssm_xzy_angle[0] = y1;
    ssm_xzy_angle[1] = y2;
    ssm_xzy_angle[2] = y3;
}
