#include "ins_task.h"
#include "QuaternionAHRS.h"
#include "exo_controller.h"
#include "dm_imu.h"
#include "arm_math.h"
#include "tim.h"

QuaternionBuf_t QuaternionBuffer;
PID_t TempCtrl = {0};

const float gravity[3] = {0, 0, 9.81f};
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
float RefTemp = 40;

static void IMU_Param_Correction(imu_cali_param_t *param, float gyro[3], float accel[3]);
static void get_INS_from_imu(INS_t *ins, float *accel, float *gyro);
static void get_INS_from_imu2(INS_t *ins, float *accel, float *gyro);

void INS_Init(void)
{
    // gEstimateKF_Init(0.01, 1000);
    BMI088_Read(&BMI088);

    exo_controller.xzy_shoulder.dm_imu.send_reg = 0x01;
    IMU_RequestData(&hcan2, 0x01, exo_controller.xzy_shoulder.dm_imu.send_reg);

    // exo_controller.xzy_shoulder.INS_shoulder.AccelLPF = 0.0085;

    exo_controller.xzy_shoulder.INS_shoulder.imu_cali_param.scale[X] = 1;
    exo_controller.xzy_shoulder.INS_shoulder.imu_cali_param.scale[Y] = 1;
    exo_controller.xzy_shoulder.INS_shoulder.imu_cali_param.scale[Z] = 1;

    exo_controller.xzy_shoulder.INS_shoulder.imu_cali_param.Yaw = 0;
    exo_controller.xzy_shoulder.INS_shoulder.imu_cali_param.Pitch = 0;
    exo_controller.xzy_shoulder.INS_shoulder.imu_cali_param.Roll = 0;
    exo_controller.xzy_shoulder.INS_shoulder.imu_cali_param.flag = 1;

    IMU_QuaternionEKF_Init(&exo_controller.xzy_shoulder.INS_shoulder.qekf_ins, 10, 0.001, 1000000 * 10, 0.9996 * 0 + 1, 0);

    exo_controller.INS_torso.imu_cali_param.scale[X] = 1;
    exo_controller.INS_torso.imu_cali_param.scale[Y] = 1;
    exo_controller.INS_torso.imu_cali_param.scale[Z] = 1;

    exo_controller.INS_torso.imu_cali_param.Yaw = 0;
    exo_controller.INS_torso.imu_cali_param.Pitch = 0;
    exo_controller.INS_torso.imu_cali_param.Roll = 0;
    exo_controller.INS_torso.imu_cali_param.flag = 1;

    IMU_QuaternionEKF_Init(&exo_controller.INS_torso.qekf_ins, 10, 0.001, 1000000 * 10, 0.9996 * 0 + 1, 0);
    // imu heat init
    PID_Init(&TempCtrl, 2000, 300, 0, 1000, 20, 0, 0, 0, 0, 0, 0, 0);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
}

void INS_Task(void)
{
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};
    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    // ins update
    if ((count % 1) == 0)
    {
        BMI088_Read(&BMI088);
        get_INS_from_imu2(&exo_controller.INS_torso, BMI088.Accel, BMI088.Gyro);

        if (exo_controller.xzy_shoulder.dm_imu.send_reg == 0x01)
            exo_controller.xzy_shoulder.dm_imu.send_reg = 0x02; // 0x01:accel, 0x02:gyro, 0x03:euler, 0x04:quaternion
        else if (exo_controller.xzy_shoulder.dm_imu.send_reg == 0x02)
            exo_controller.xzy_shoulder.dm_imu.send_reg = 0x01;
        IMU_RequestData(&hcan2, 0x01, exo_controller.xzy_shoulder.dm_imu.send_reg);
        get_INS_from_imu(&exo_controller.xzy_shoulder.INS_shoulder,
                         exo_controller.xzy_shoulder.dm_imu.accel, exo_controller.xzy_shoulder.dm_imu.gyro);

        // 创建结构体实例
        Quaternion torso_q_struct;
        Quaternion humerus_q_struct;
        EulerAnglesXZY result_angles_struct;

        // 将数组数据填充到结构体中
        torso_q_struct.w = exo_controller.INS_torso.q[0];
        torso_q_struct.x = exo_controller.INS_torso.q[1];
        torso_q_struct.y = exo_controller.INS_torso.q[2];
        torso_q_struct.z = exo_controller.INS_torso.q[3];
        humerus_q_struct.w = exo_controller.xzy_shoulder.INS_shoulder.q[0];
        humerus_q_struct.x = exo_controller.xzy_shoulder.INS_shoulder.q[1];
        humerus_q_struct.y = exo_controller.xzy_shoulder.INS_shoulder.q[2];
        humerus_q_struct.z = exo_controller.xzy_shoulder.INS_shoulder.q[3];

        get_shoulder_angles_wrt_torso(&torso_q_struct, &humerus_q_struct, &exo_controller.shoulder_xzy_angle_wrt_torso);
        exo_controller.shoulder_xzy_angle_wrt_torso.phi_x *= RADIAN_COEF;
        exo_controller.shoulder_xzy_angle_wrt_torso.psi_z *= RADIAN_COEF;
        exo_controller.shoulder_xzy_angle_wrt_torso.theta_y *= RADIAN_COEF;
    }

    // temperature control
    if ((count % 2) == 0)
    {
        // 500hz
        IMU_Temperature_Ctrl();
        if (GlobalDebugMode == IMU_HEAT_DEBUG)
            Serial_Debug(&huart1, 1, RefTemp, BMI088.Temperature, TempCtrl.Output / 1000.0f, TempCtrl.Pout / 1000.0f, TempCtrl.Iout / 1000.0f, TempCtrl.Dout / 1000.0f);
    }

    if ((count % 1000) == 0)
    {
        // 200hz
    }

    count++;
}

static void get_INS_from_imu(INS_t *ins, float *accel, float *gyro)
{
    ins->Accel[X] = -accel[Y];
    ins->Accel[Y] = -accel[Z];
    ins->Accel[Z] = accel[X];
    ins->Gyro[X] = -gyro[Y];
    ins->Gyro[Y] = -gyro[Z];
    ins->Gyro[Z] = gyro[X];

    IMU_Param_Correction(&ins->imu_cali_param, ins->Gyro, ins->Accel);
    ins->atanxz = -atan2f(ins->Accel[X], ins->Accel[Z]) * 180 / PI;
    ins->atanyz = atan2f(ins->Accel[Y], ins->Accel[Z]) * 180 / PI;

    // Quaternion_AHRS_UpdateIMU(ins->Gyro[X], ins->Gyro[Y], ins->Gyro[Z], ins->Accel[X], ins->Accel[Y], ins->Accel[Z], 0, 0, 0, dt);
    IMU_QuaternionEKF_Update(&ins->qekf_ins, ins->Gyro[X], ins->Gyro[Y], ins->Gyro[Z], ins->Accel[X], ins->Accel[Y], ins->Accel[Z], dt);

    float *q = &ins->qekf_ins.q;
    memcpy(ins->q, ins->qekf_ins.q, sizeof(ins->qekf_ins.q));
    ins->xzy_order_angle[1] = -asinf(2 * (q[1] * q[2] - q[0] * q[3])) * 57.295779513f;
    ins->xzy_order_angle[2] = atan2f(2.0f * (q[1] * q[3] + q[0] * q[2]), 1 - 2.0f * (q[2] * q[2] + q[3] * q[3])) * 57.295779513f;
    ins->xzy_order_angle[0] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1 - 2.0f * (q[1] * q[1] - q[3] * q[3])) * 57.295779513f;

    BodyFrameToEarthFrame(xb, ins->xn, ins->q);
    BodyFrameToEarthFrame(yb, ins->yn, ins->q);
    BodyFrameToEarthFrame(zb, ins->zn, ins->q);

    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, ins->q);
    for (uint8_t i = 0; i < 3; i++)
        ins->MotionAccel_b[i] = (ins->Accel[i] - gravity_b[i]) * dt / (ins->AccelLPF + dt) + ins->MotionAccel_b[i] * ins->AccelLPF / (ins->AccelLPF + dt);
    BodyFrameToEarthFrame(ins->MotionAccel_b, ins->MotionAccel_n, ins->q);

    memcpy(ins->Gyro, ins->qekf_ins.Gyro, sizeof(ins->qekf_ins.Gyro));
    memcpy(ins->Accel, ins->qekf_ins.Accel, sizeof(ins->qekf_ins.Accel));
    memcpy(ins->q, ins->qekf_ins.q, sizeof(ins->qekf_ins.q));
    ins->Yaw = ins->qekf_ins.Yaw;
    ins->Pitch = ins->qekf_ins.Pitch;
    // ins->Roll = ins->qekf_ins.Roll;
    ins->YawTotalAngle = ins->qekf_ins.YawTotalAngle;

    InsertQuaternionFrame(&QuaternionBuffer, ins->q, ins->MotionAccel_n, INS_GetTimeline());

    // Get_EulerAngle(AHRS.q);
}

static void get_INS_from_imu2(INS_t *ins, float *accel, float *gyro)
{
    ins->Accel[X] = -accel[Z];
    ins->Accel[Y] = accel[Y];
    ins->Accel[Z] = accel[X];
    ins->Gyro[X] = -gyro[Z];
    ins->Gyro[Y] = gyro[Y];
    ins->Gyro[Z] = gyro[X];

    IMU_Param_Correction(&ins->imu_cali_param, ins->Gyro, ins->Accel);
    ins->atanxz = -atan2f(ins->Accel[X], ins->Accel[Z]) * 180 / PI;
    ins->atanyz = atan2f(ins->Accel[Y], ins->Accel[Z]) * 180 / PI;

    // Quaternion_AHRS_UpdateIMU(ins->Gyro[X], ins->Gyro[Y], ins->Gyro[Z], ins->Accel[X], ins->Accel[Y], ins->Accel[Z], 0, 0, 0, dt);
    IMU_QuaternionEKF_Update(&ins->qekf_ins, ins->Gyro[X], ins->Gyro[Y], ins->Gyro[Z], ins->Accel[X], ins->Accel[Y], ins->Accel[Z], dt);

    float *q = &ins->qekf_ins.q;
    memcpy(ins->q, ins->qekf_ins.q, sizeof(ins->qekf_ins.q));
    ins->xzy_order_angle[1] = -asinf(2 * (q[1] * q[2] - q[0] * q[3])) * 57.295779513f;
    ins->xzy_order_angle[2] = atan2f(2.0f * (q[1] * q[3] + q[0] * q[2]), 1 - 2.0f * (q[2] * q[2] + q[3] * q[3])) * 57.295779513f;
    ins->xzy_order_angle[0] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1 - 2.0f * (q[1] * q[1] - q[3] * q[3])) * 57.295779513f;

    BodyFrameToEarthFrame(xb, ins->xn, ins->q);
    BodyFrameToEarthFrame(yb, ins->yn, ins->q);
    BodyFrameToEarthFrame(zb, ins->zn, ins->q);

    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, ins->q);
    for (uint8_t i = 0; i < 3; i++)
        ins->MotionAccel_b[i] = (ins->Accel[i] - gravity_b[i]) * dt / (ins->AccelLPF + dt) + ins->MotionAccel_b[i] * ins->AccelLPF / (ins->AccelLPF + dt);
    BodyFrameToEarthFrame(ins->MotionAccel_b, ins->MotionAccel_n, ins->q);

    memcpy(ins->Gyro, ins->qekf_ins.Gyro, sizeof(ins->qekf_ins.Gyro));
    memcpy(ins->Accel, ins->qekf_ins.Accel, sizeof(ins->qekf_ins.Accel));
    memcpy(ins->q, ins->qekf_ins.q, sizeof(ins->qekf_ins.q));
    ins->Yaw = ins->qekf_ins.Yaw;
    ins->Pitch = ins->qekf_ins.Pitch;
    // ins->Roll = ins->qekf_ins.Roll;
    ins->YawTotalAngle = ins->qekf_ins.YawTotalAngle;

    InsertQuaternionFrame(&QuaternionBuffer, ins->q, ins->MotionAccel_n, INS_GetTimeline());

    // Get_EulerAngle(AHRS.q);
}

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}

void InsertQuaternionFrame(QuaternionBuf_t *qBuf, float *q, float *motion_acc_n, uint32_t time_stamp_ms)
{
    if (qBuf->LatestNum == Q_FRAME_LEN - 1)
        qBuf->LatestNum = 0;
    else
        qBuf->LatestNum++;

    qBuf->qFrame[qBuf->LatestNum].TimeStamp_ms = time_stamp_ms;
    for (uint16_t i = 0; i < 4; i++)
        qBuf->qFrame[qBuf->LatestNum].q[i] = q[i];
    for (uint16_t i = 0; i < 3; i++)
        qBuf->qFrame[qBuf->LatestNum].MotionAccel_n[i] = motion_acc_n[i];
}

uint16_t FindTimeMatchFrame(QuaternionBuf_t *qBuf, uint32_t match_time_stamp_ms)
{
    int min_time_error = abs(qBuf->qFrame[0].TimeStamp_ms - match_time_stamp_ms);
    uint16_t num = 0;
    for (uint16_t i = 0; i < Q_FRAME_LEN; i++)
    {
        if (abs(qBuf->qFrame[i].TimeStamp_ms - match_time_stamp_ms) < min_time_error)
        {
            min_time_error = abs(qBuf->qFrame[i].TimeStamp_ms - match_time_stamp_ms);
            num = i;
        }
    }
    return num;
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

static void IMU_Param_Correction(imu_cali_param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

void IMU_Temperature_Ctrl(void)
{
    PID_Calculate(&TempCtrl, BMI088.Temperature, 40 + 0 * float_constrain(BMI088.TempWhenCali, 37, 42));

    TIM_Set_PWM(&htim10, TIM_CHANNEL_1, float_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX));
}

/**
 * @brief 计算四元数的共轭。
 * @param[in]   q_in    输入的四元数
 * @param[out]  q_out   存储共轭结果的四元数
 */
static void quaternion_conjugate(const Quaternion *q_in, Quaternion *q_out)
{
    q_out->w = q_in->w;
    q_out->x = -q_in->x;
    q_out->y = -q_in->y;
    q_out->z = -q_in->z;
}

/**
 * @brief 计算两个四元数的乘法 (result = q1 * q2)。
 * @param[in]   q1      左侧的四元数
 * @param[in]   q2      右侧的四元数
 * @param[out]  result  存储乘法结果的四元数
 */
static void quaternion_multiply(const Quaternion *q1, const Quaternion *q2, Quaternion *result)
{
    result->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    result->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
}

/**
 * @brief 主函数：从躯干和肱骨的姿态四元数计算肩关节的相对旋转欧拉角。
 */
void get_shoulder_angles_wrt_torso(const Quaternion *q_torso, const Quaternion *q_humerus, EulerAnglesXZY *angles)
{
    // --- 步骤 1: 计算肩关节的相对旋转四元数 (无变化) ---
    Quaternion q_torso_conj;
    Quaternion q_shoulder;

    quaternion_conjugate(q_torso, &q_torso_conj);
    quaternion_multiply(&q_torso_conj, q_humerus, &q_shoulder);

    // --- 步骤 2: 将相对旋转四元数转换为 X-Z-Y 欧拉角 (已更新) ---
    float w = q_shoulder.w; // 对应您的 q
    float x = q_shoulder.x; // 对应您的 q[1]
    float y = q_shoulder.y; // 对应您的 q[2]
    float z = q_shoulder.z; // 对应您的 q[3]

    // 根据您的Z轴公式 `asinf(2*(w*z - x*y))` 来确定万向节死锁条件
    float test_val = 2.0f * (w * z - x * y);
    const float GIMBAL_LOCK_THRESHOLD = 0.99999f;

    if (fabsf(test_val) > GIMBAL_LOCK_THRESHOLD)
    {
        // --- 万向节死锁情况 ---
        // 当绕Z轴旋转接近 +/- 90度时发生
        angles->psi_z = (test_val > 0.0f) ? (PI / 2.0f) : (-PI / 2.0f); // Z轴旋转
        angles->phi_x = 2.0f * atan2f(x, w);                            // X轴旋转 (使用稳健的计算方式)
        angles->theta_y = 0.0f;                                         // Y轴旋转 (按惯例设为0)
    }
    else
    {
        // --- 正常情况：使用您提供的公式 ---

        // Z轴旋转 (psi)，对应您的 angle[1]
        // 您的公式: -asinf(2 * (x * y - w * z))
        // 等价于:    asinf(2 * (w * z - x * y))
        angles->psi_z = asinf(test_val);

        // Y轴旋转 (theta)，对应您的 angle[2]
        // 您的公式: atan2f(2.0f * (x * z + w * y), 1 - 2.0f * (y * y + z * z))
        angles->theta_y = atan2f(2.0f * (x * z + w * y), 1.0f - 2.0f * (y * y + z * z));

        // X轴旋转 (phi)，对应您的 angle
        // 您的公式: atan2f(2.0f * (w * x + y * z), 1 - 2.0f * (x * x - z * z))
        angles->phi_x = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x - z * z));
    }
    // ins->xzy_order_angle[1] = -asinf(2 * (q[1] * q[2] - q[0] * q[3])) * 57.295779513f;
    // ins->xzy_order_angle[2] = atan2f(2.0f * (q[1] * q[3] + q[0] * q[2]), 1 - 2.0f * (q[2] * q[2] + q[3] * q[3])) * 57.295779513f;
    // ins->xzy_order_angle[0] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1 - 2.0f * (q[1] * q[1] - q[3] * q[3])) * 57.295779513f;
}
