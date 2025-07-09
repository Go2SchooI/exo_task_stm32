#ifndef __DM_IMU_H
#define __DM_IMU_H

#include "stm32f4xx_hal.h"

#define ACCEL_CAN_MAX (58.8f)
#define ACCEL_CAN_MIN (-58.8f)
#define GYRO_CAN_MAX (34.88f)
#define GYRO_CAN_MIN (-34.88f)
#define PITCH_CAN_MAX (90.0f)
#define PITCH_CAN_MIN (-90.0f)
#define ROLL_CAN_MAX (180.0f)
#define ROLL_CAN_MIN (-180.0f)
#define YAW_CAN_MAX (180.0f)
#define YAW_CAN_MIN (-180.0f)
#define TEMP_MIN (0.0f)
#define TEMP_MAX (60.0f)
#define Quaternion_MIN (-1.0f)
#define Quaternion_MAX (1.0f)

typedef struct
{
	uint8_t send_reg; // 0x01:accel, 0x02:gyro, 0x03:euler, 0x04:quaternion

	float pitch;
	float roll;
	float yaw;

	float gyro[3];
	float accel[3];

	float q[4];

	float cur_temp;

	float xzy_order_angle[3]; // xzy order angle
	float xzy_order_total_angle[3];

	uint8_t tmp_rx_data[8];
} dm_imu_t;

void IMU_UpdateData(dm_imu_t *imu, uint8_t *pData);
void IMU_RequestData(CAN_HandleTypeDef *hcan, uint16_t can_id, uint8_t reg);
#endif