#include "ins_task.h"
#include "QuaternionAHRS.h"
#include "includes.h"
#include "exo_controller.h"
#include "QuaternionEKF.h"
#include "dm_imu.h"
#include "tim.h"

INS_t INS;
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

    IMU_QuaternionEKF_Init(10, 0.001, 1000000 * 10, 0.9996 * 0 + 1, 0);
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
        // INS.Accel[X] = -BMI088.Accel[Y];
        // INS.Accel[Y] = -BMI088.Accel[Z];
        // INS.Accel[Z] = BMI088.Accel[X];
        // INS.Gyro[X] = -BMI088.Gyro[Y];
        // INS.Gyro[Y] = -BMI088.Gyro[Z];
        // INS.Gyro[Z] = BMI088.Gyro[X];

        if (exo_controller.xzy_shoulder.dm_imu.send_reg == 0x01)
            exo_controller.xzy_shoulder.dm_imu.send_reg = 0x02; // 0x01:accel, 0x02:gyro, 0x03:euler, 0x04:quaternion
        else if (exo_controller.xzy_shoulder.dm_imu.send_reg == 0x02)
            exo_controller.xzy_shoulder.dm_imu.send_reg = 0x01;
        IMU_RequestData(&hcan2, 0x01, exo_controller.xzy_shoulder.dm_imu.send_reg);
        get_INS_from_imu(&exo_controller.xzy_shoulder.INS_shoulder,
                         exo_controller.xzy_shoulder.dm_imu.accel, exo_controller.xzy_shoulder.dm_imu.gyro);
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
    IMU_QuaternionEKF_Update(ins->Gyro[X], ins->Gyro[Y], ins->Gyro[Z], ins->Accel[X], ins->Accel[Y], ins->Accel[Z], dt);

    float *q = QEKF_INS.q;
    ins->xzy_order_angle[1] = -asinf(2 * (q[1] * q[2] - q[0] * q[3])) * 57.295779513f;
    ins->xzy_order_angle[2] = atan2f(2.0f * (q[1] * q[3] + q[0] * q[2]), 1 - 2.0f * (q[2] * q[2] + q[3] * q[3])) * 57.295779513f;
    ins->xzy_order_angle[0] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1 - 2.0f * (q[1] * q[1] - q[3] * q[3])) * 57.295779513f;

    BodyFrameToEarthFrame(xb, ins->xn, ins->q);
    BodyFrameToEarthFrame(yb, ins->yn, ins->q);
    BodyFrameToEarthFrame(zb, ins->zn, ins->q);

    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
    for (uint8_t i = 0; i < 3; i++)
        ins->MotionAccel_b[i] = (ins->Accel[i] - gravity_b[i]) * dt / (ins->AccelLPF + dt) + ins->MotionAccel_b[i] * ins->AccelLPF / (ins->AccelLPF + dt);
    BodyFrameToEarthFrame(ins->MotionAccel_b, ins->MotionAccel_n, ins->q);

    memcpy(ins->Gyro, QEKF_INS.Gyro, sizeof(QEKF_INS.Gyro));
    memcpy(ins->Accel, QEKF_INS.Accel, sizeof(QEKF_INS.Accel));
    memcpy(ins->q, QEKF_INS.q, sizeof(QEKF_INS.q));
    ins->Yaw = QEKF_INS.Yaw;
    ins->Pitch = QEKF_INS.Pitch;
    // ins->Roll = QEKF_INS.Roll;
    ins->YawTotalAngle = QEKF_INS.YawTotalAngle;

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
