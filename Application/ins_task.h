#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "stdint.h"
#include "includes.h"

#define INS_TASK_PERIOD 1

#define INS_GetTimeline HAL_GetTick

#define X 0
#define Y 1
#define Z 2

typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} imu_cali_param_t;

typedef __packed struct
{
    float q[4]; // 四元数估计值

    float Gyro[3];
    float Accel[3];
    float MotionAccel_b[3];
    float MotionAccel_n[3];

    float AccelLPF;

    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;

    imu_cali_param_t imu_cali_param;
    float xzy_order_angle[3]; // xzy order angle
} INS_t;

typedef struct
{
    float q[4];
    float MotionAccel_n[3];
    uint32_t TimeStamp_ms;
} QuaternionFrame_t;

#define Q_FRAME_LEN 50
typedef struct
{
    QuaternionFrame_t qFrame[Q_FRAME_LEN];
    uint16_t LatestNum;
} QuaternionBuf_t;

extern INS_t INS;
extern float RefTemp;
extern QuaternionBuf_t QuaternionBuffer;

void INS_Init(void);
void INS_Task(void);
void IMU_Temperature_Ctrl(void);

void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
void InsertQuaternionFrame(QuaternionBuf_t *qBuf, float *q, float *motion_acc_n, uint32_t time_stamp_ms);
uint16_t FindTimeMatchFrame(QuaternionBuf_t *qBuf, uint32_t match_time_stamp_ms);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

#endif
