#ifndef _MOTOR_H
#define _MOTOR_H

#include <stdint.h>
#include "controller.h"
#include "can.h"

#define ENCODERCOEF 0.0439453125f
#define RADIAN_COEF 57.295779513f

// CAN Transmit ID
#define CAN_Transmit_1_4_ID 0x200
#define CAN_Transmit_5_8_ID 0x1ff
#define CAN_Transmit_6020_ID 0x2ff

// CAN Receive ID
#define CAN_Receive_1_ID 0x201
#define CAN_Receive_2_ID 0x202
#define CAN_Receive_3_ID 0x203
#define CAN_Receive_4_ID 0x204
#define CAN_Receive_5_ID 0x205
#define CAN_Receive_6_ID 0x206
#define CAN_Receive_7_ID 0x207
#define CAN_Receive_8_ID 0x208

#define Motor_mode 0 // 设置模式为何种模式，为0为MIT模式，为1为位置速度模式，为2为速度模式

#define ENABLE_MOTOR 1    // 使能电机
#define DISABLE_MOTOR 0   // 禁用电机
#define SAVE_ZERO_MOTOR 2 // 保存零点

#define P_MIN -12.5  // 位置最小值
#define P_MAX 12.5   // 位置最大值
#define V_MIN -45    // 速度最小值
#define V_MAX 45     // 速度最大值
#define KP_MIN 0.0   // Kp最小值
#define KP_MAX 500.0 // Kp最大值
#define KD_MIN 0.0   // Kd最小值
#define KD_MAX 5.0   // Kd最大值
#define T_MIN -18    // 转矩最小值
#define T_MAX 18     // 转矩最大值

#define NEGATIVE 1

#define LK8008_CURRENT_COEF 402.7f

/*moto information receive from CAN*/
typedef struct
{
  float reduction_ratio;
  float velocity_in_radps;
  int raw_velocity; // abs velocity range:[-8191,8191]
  int16_t velocity_in_rpm;
  float Real_Current;
  uint8_t Temperature;
  uint8_t Direction;

  float Ke;
  int raw_torque;
  float torque_in_Nm; // abs torque range:[-12.5,12.5]

  float Angle; // abs angle range:[0,8191]
  float angle_in_degree;
  uint16_t RawAngle; // abs angle range:[0,8191]
  int dm_raw_angle;
  float output_angle;
  uint16_t last_angle;
  int16_t offset_angle;
  int16_t zero_offset;
  int32_t round_cnt;
  int32_t total_angle;
  double total_output_angle;

  int16_t angle_buf[5];
  int32_t msg_cnt;

  uint16_t CAN_ID;
  uint8_t Err; // 错误码

  float Output;
  float max_out;

  PID_t PID_Torque;
  PID_t PID_Velocity;
  PID_t PID_Angle;
} motor_info;

float Motor_Torque_Calculate(motor_info *motor, float torque, float target_torque);
float Motor_Speed_Calculate(motor_info *motor, float velocity, float target_speed);
float Motor_Angle_Calculate(motor_info *motor, float angle, float velocity, float target_angle);

void Get_Motor_Info(motor_info *ptr, uint8_t *aData);
void Get_Motor_Offset(motor_info *ptr, uint8_t *aData);
void Get_LK_Info(motor_info *ptr, uint8_t *aData);
void Get_LK_Offset(motor_info *ptr, uint8_t *aData);
void Get_DM_Info(motor_info *ptr, uint8_t *aData);
void Get_DM_Offset(motor_info *ptr, uint8_t *aData);

HAL_StatusTypeDef Send_Motor_Current_1_4(CAN_HandleTypeDef *_hcan, int16_t c1, int16_t c2, int16_t c3, int16_t c4);
HAL_StatusTypeDef Send_Motor_Current_5_8(CAN_HandleTypeDef *_hcan, int16_t c1, int16_t c2, int16_t c3, int16_t c4);
HAL_StatusTypeDef Send_LK_Current_Single(CAN_HandleTypeDef *_hcan, int8_t ID, int16_t c);
HAL_StatusTypeDef Send_LK_Current(CAN_HandleTypeDef *_hcan, int16_t c1, int16_t c2, int16_t c3, int16_t c4);

HAL_StatusTypeDef Send_DM_MIT_Command(CAN_HandleTypeDef *_hcan, uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);
uint8_t DM_CANx_SendStdData(CAN_HandleTypeDef *_hcan, uint16_t id, uint8_t cmd, uint16_t len);

int16_t LK8008_Current_Solver(float torque);

#endif
