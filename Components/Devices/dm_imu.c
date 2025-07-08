#include "dm_imu.h"
#include "can.h"
#include "motor.h"
#include "QuaternionAHRS.h"

void IMU_RequestData(CAN_HandleTypeDef *hcan, uint16_t can_id, uint8_t reg)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t cmd[4] = {(uint8_t)can_id, (uint8_t)(can_id >> 8), reg, 0xCC};
	uint32_t returnBox;
	tx_header.DLC = 4;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.StdId = 0x6FF;

	if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 1)
	{
		HAL_CAN_AddTxMessage(hcan, &tx_header, cmd, &returnBox);
	}
}

void IMU_UpdateAccel(dm_imu_t *imu, uint8_t *pData)
{
	uint16_t accel[3];

	accel[0] = pData[3] << 8 | pData[2];
	accel[1] = pData[5] << 8 | pData[4];
	accel[2] = pData[7] << 8 | pData[6];

	imu->accel[0] = uint_to_float(accel[0], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
	imu->accel[1] = uint_to_float(accel[1], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
	imu->accel[2] = uint_to_float(accel[2], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
}

void IMU_UpdateGyro(dm_imu_t *imu, uint8_t *pData)
{
	uint16_t gyro[3];

	gyro[0] = pData[3] << 8 | pData[2];
	gyro[1] = pData[5] << 8 | pData[4];
	gyro[2] = pData[7] << 8 | pData[6];

	imu->gyro[0] = uint_to_float(gyro[0], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
	imu->gyro[1] = uint_to_float(gyro[1], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
	imu->gyro[2] = uint_to_float(gyro[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
}

void IMU_UpdateEuler(dm_imu_t *imu, uint8_t *pData)
{
	int euler[3];

	euler[0] = pData[3] << 8 | pData[2];
	euler[1] = pData[5] << 8 | pData[4];
	euler[2] = pData[7] << 8 | pData[6];

	imu->pitch = uint_to_float(euler[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
	imu->yaw = uint_to_float(euler[1], YAW_CAN_MIN, YAW_CAN_MAX, 16);
	imu->roll = uint_to_float(euler[2], ROLL_CAN_MIN, ROLL_CAN_MAX, 16);
}

void IMU_UpdateQuaternion(dm_imu_t *imu, uint8_t *pData)
{
	int w = pData[1] << 6 | ((pData[2] & 0xF8) >> 2);
	int x = (pData[2] & 0x03) << 12 | (pData[3] << 4) | ((pData[4] & 0xF0) >> 4);
	int y = (pData[4] & 0x0F) << 10 | (pData[5] << 2) | (pData[6] & 0xC0) >> 6;
	int z = (pData[6] & 0x3F) << 8 | pData[7];

	imu->q[0] = uint_to_float(w, Quaternion_MIN, Quaternion_MAX, 14);
	imu->q[1] = uint_to_float(x, Quaternion_MIN, Quaternion_MAX, 14);
	imu->q[2] = uint_to_float(y, Quaternion_MIN, Quaternion_MAX, 14);
	imu->q[3] = uint_to_float(z, Quaternion_MIN, Quaternion_MAX, 14);

	get_xzy_order_angle(imu);
}

void IMU_UpdateData(dm_imu_t *imu, uint8_t *pData)
{
	memcpy(imu->tmp_rx_data, pData, 8);
	switch (pData[0])
	{
	case 1:
		IMU_UpdateAccel(imu, pData);
		break;
	case 2:
		IMU_UpdateGyro(imu, pData);
		break;
	case 3:
		IMU_UpdateEuler(imu, pData);
		break;
	case 4:
		IMU_UpdateQuaternion(imu, pData);
		break;
	}
}

void get_xzy_order_angle(dm_imu_t *imu)
{
	static float tmp_yaw, tmp_pitch, tmp_roll;
	static uint16_t angle0_count, angle1_count, angle2_count;
	static float last_angle0, last_angle1, last_angle2;
	float *q = imu->q;

	// float norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	// if (fabsf(norm - 1.0f) > 0.001f)
	// {
	// 	q[0] /= norm;
	// 	q[1] /= norm;
	// 	q[2] /= norm;
	// 	q[3] /= norm;
	// }

	tmp_yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
	tmp_pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
	tmp_roll = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2])) * 57.295779513f;

	// qh2_subs = -asin(T_e(1, 2));
	// qh3_subs = atan2(T_e(1, 3), T_e(1, 1));
	// qh1_subs = atan2(T_e(3, 2), T_e(2, 2));

	// IMU 安装变换矩阵 P：x' =  z,  y' = -x,  z' = -y
	// T = [ 0, -1, 0;
	// 	  0, 0, -1;
	// 	  1, 0, 0 ];
	// 考虑安装安装偏转后的旋转矩阵 R_inst
	// R_inst = R_std * T'
	imu->xzy_order_angle[1] = -asinf(2.0f * (q[2] * q[2] + q[3] * q[3]) - 1.0f) * 57.295779513f;

	imu->xzy_order_angle[2] = atan2f(2.0f * (q[0] * q[3] - q[1] * q[2]), 2.0f * (q[0] * q[2] + q[1] * q[3])) * 57.295779513f;
	imu->xzy_order_angle[0] = atan2f(2.0f * (q[0] * q[2] - q[1] * q[3]), -2.0f * (q[0] * q[3] + q[2] * q[1])) * 57.295779513f;

	if (imu->xzy_order_angle[0] - last_angle0 > 180.0f)
		angle0_count--;
	else if (imu->xzy_order_angle[0] - last_angle0 < -180.0f)
		angle0_count++;

	if (imu->xzy_order_angle[1] - last_angle1 > 180.0f)
		angle1_count--;
	else if (imu->xzy_order_angle[1] - last_angle1 < -180.0f)
		angle1_count++;

	if (imu->xzy_order_angle[2] - last_angle2 > 180.0f)
		angle2_count--;
	else if (imu->xzy_order_angle[2] - last_angle2 < -180.0f)
		angle2_count++;

	imu->xzy_order_total_angle[0] = 360.0f * angle0_count + imu->xzy_order_angle[0];
	imu->xzy_order_total_angle[1] = 360.0f * angle1_count + imu->xzy_order_angle[1];
	imu->xzy_order_total_angle[2] = 360.0f * angle2_count + imu->xzy_order_angle[2];

	last_angle0 = imu->xzy_order_angle[0];
	last_angle1 = imu->xzy_order_angle[1];
	last_angle2 = imu->xzy_order_angle[2];
}
