#include "motor.h"
#include "includes.h"

float debugNum[5] = {0, 0, 0, 0, 0}; // debugÓÃ

float Motor_Torque_Calculate(motor_info *motor, float torque, float target_torque)
{
    // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
    PID_Calculate(&motor->PID_Torque, torque, target_torque);

    if (motor->Direction != NEGATIVE)
        motor->Output = motor->PID_Torque.Output + motor->Ke * motor->velocity_in_rpm;
    else
        motor->Output = motor->PID_Torque.Output - motor->Ke * motor->velocity_in_rpm;
    // ï¿½ï¿½ï¿½ï¿½Ş·ï¿?
    motor->Output = float_constrain(motor->Output, -motor->max_out, motor->max_out);

    return motor->Output;
}

float Motor_Speed_Calculate(motor_info *motor, float velocity, float target_speed)
{
    PID_Calculate(&motor->PID_Velocity, velocity, target_speed);
    motor->Output = motor->PID_Velocity.Output;
    motor->Output = float_constrain(motor->Output, -motor->max_out, motor->max_out);

    return motor->Output;
}

float Motor_Angle_Calculate(motor_info *motor, float angle, float velocity, float target_angle)
{
    PID_Calculate(&motor->PID_Angle, angle, target_angle);
    Motor_Speed_Calculate(motor, velocity, motor->PID_Angle.Output);
    motor->Output = float_constrain(motor->Output, -motor->max_out, motor->max_out);

    return motor->Output;
}

/* -------------------------------- DJI motor ------------------------------- */
/**
 * @Func	    void get_moto_info(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
 * @Brief      process data received from CAN
 * @Param	    motor_info *ptr  CAN_HandleTypeDef *_hcan
 * @Retval	    None
 * @Date       2019/11/5
 **/
void Get_Motor_Info(motor_info *ptr, uint8_t *aData)
{
    // ï¿½ï¿½ï¿½C620ï¿½ï¿½ï¿½ï¿½Ö²ï¿?
    if (ptr->Direction != NEGATIVE)
    {
        ptr->RawAngle = (uint16_t)(aData[0] << 8 | aData[1]);
        ptr->velocity_in_rpm = (int16_t)(aData[2] << 8 | aData[3]);
    }
    else
    {
        ptr->RawAngle = 8191 - (uint16_t)(aData[0] << 8 | aData[1]);
        ptr->velocity_in_rpm = -(int16_t)(aData[2] << 8 | aData[3]);
    }

    ptr->Real_Current = (aData[4] << 8 | aData[5]) * 5.0f / 16384.0f;
    ptr->Temperature = aData[6];

    // ï¿½ï¿½ï¿½ï¿½È¦ï¿½ï¿½
    if (ptr->RawAngle - ptr->last_angle > 4096)
        ptr->round_cnt--;
    else if (ptr->RawAngle - ptr->last_angle < -4096)
        ptr->round_cnt++;

    // ×ªï¿½ï¿½ï¿½ï¿½Ğµï¿½ï¿½ï¿?
    ptr->Angle = loop_float_constrain(ptr->RawAngle + ptr->zero_offset, -4095.5, 4095.5);

    // ×ªï¿½ï¿½ï¿½Ç¶Èµï¿½Î»Îªï¿½ï¿½
    ptr->angle_in_degree = ptr->Angle * 0.0439507f;

    // ï¿½ï¿½ï¿½ï¿½ï¿½Ü½Ç¶ï¿½
    ptr->total_angle = ptr->round_cnt * 8192 + ptr->RawAngle - ptr->offset_angle;

    ptr->last_angle = ptr->RawAngle; // update last_angle
}

/*this function should be called after system+can init */
void Get_Motor_Offset(motor_info *ptr, uint8_t *aData)
{
    ptr->RawAngle = (uint16_t)(aData[0] << 8 | aData[1]);
    ptr->offset_angle = ptr->RawAngle;
}

/**
  * @Func	    void Send_Motor_Current(CAN_HandleTypeDef* hcan,
                                    int16_t c1, int16_t c2, int16_t c3, int16_t c4)
  * @Brief
  * @Param	    cx x=1,2,3,4
  * @Retval	    None
  * @Date       2019/11/5
 **/
HAL_StatusTypeDef Send_Motor_Current_1_4(CAN_HandleTypeDef *_hcan,
                                         int16_t c1, int16_t c2, int16_t c3, int16_t c4)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    // ï¿½ï¿½ï¿½Ã·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ°ï¿½ï¿½ï¿½IDï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½IDÎªï¿½Ø¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Î?Ä¬ï¿½Ï£ï¿½Í¨ï¿½Ã£ï¿½ï¿½ï¿½ï¿?
    TX_MSG.StdId = CAN_Transmit_1_4_ID;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;

    // ï¿½ï¿½ï¿½ï¿½C620ï¿½ï¿½ï¿½Ğ?ï¿½ï¿½ï¿½ï¿½ï¿½Ãµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö?
    CAN_Send_Data[0] = (c1 >> 8);
    CAN_Send_Data[1] = c1;
    CAN_Send_Data[2] = (c2 >> 8);
    CAN_Send_Data[3] = c2;
    CAN_Send_Data[4] = (c3 >> 8);
    CAN_Send_Data[5] = c3;
    CAN_Send_Data[6] = (c4 >> 8);
    CAN_Send_Data[7] = c4;

    // CANï¿½ï¿½ï¿½İ°ï¿½ï¿½ï¿½ï¿½ï¿½
    return HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

HAL_StatusTypeDef Send_Motor_Current_5_8(CAN_HandleTypeDef *_hcan,
                                         int16_t c1, int16_t c2, int16_t c3, int16_t c4)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = CAN_Transmit_5_8_ID;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    CAN_Send_Data[0] = (c1 >> 8);
    CAN_Send_Data[1] = c1;
    CAN_Send_Data[2] = (c2 >> 8);
    CAN_Send_Data[3] = c2;
    CAN_Send_Data[4] = (c3 >> 8);
    CAN_Send_Data[5] = c3;
    CAN_Send_Data[6] = (c4 >> 8);
    CAN_Send_Data[7] = c4;

    return HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

/* -------------------------------- LK motor -------------------------------- */
void Get_LK_Info(motor_info *ptr, uint8_t *aData)
{
    if (ptr->Direction != NEGATIVE)
    {
        ptr->RawAngle = (uint16_t)(aData[7] << 8 | aData[6]);
        ptr->velocity_in_rpm = (int16_t)(aData[5] << 8 | aData[4]) * 0.159154943f;
    }
    else
    {
        ptr->RawAngle = 32767 - (uint16_t)(aData[7] << 8 | aData[6]);
        ptr->velocity_in_rpm = -(int16_t)(aData[5] << 8 | aData[4]) * 0.159154943f;
    }

    ptr->Real_Current = (int16_t)(aData[3] << 8 | aData[2]);
    ptr->Temperature = aData[1];

    if (ptr->RawAngle - ptr->last_angle > 32767)
        ptr->round_cnt--;
    else if (ptr->RawAngle - ptr->last_angle < -32767)
        ptr->round_cnt++;

    ptr->Angle = loop_float_constrain(ptr->RawAngle - ptr->zero_offset, -32767, 32767);

    ptr->angle_in_degree = ptr->Angle * 0.00549324788281f;

    ptr->total_angle = ptr->round_cnt * 65535 + ptr->RawAngle - ptr->offset_angle;

    ptr->last_angle = ptr->RawAngle; // update last_angle
}

/*this function should be called after system+can init */
void Get_LK_Offset(motor_info *ptr, uint8_t *aData)
{
    ptr->RawAngle = (uint16_t)(aData[7] << 8 | aData[6]);
    ptr->offset_angle = ptr->RawAngle;
    ptr->last_angle = ptr->RawAngle; // update last_angle
}

/**
  * @Func	    void Send_RMD_Current(CAN_HandleTypeDef* hcan,
                                    int16_t c1, int16_t c2, int16_t c3, int16_t c4)
  * @Brief
  * @Param	    cx x=1,2,3,4
  * @Retval	    None
  * @Date       2019/11/5
 **/
HAL_StatusTypeDef Send_LK_Current(CAN_HandleTypeDef *_hcan,
                                  int16_t c1, int16_t c2, int16_t c3, int16_t c4)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    // ï¿½ï¿½ï¿½Ã·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ°ï¿½ï¿½ï¿½IDï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½IDÎªï¿½Ø¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Î?Ä¬ï¿½Ï£ï¿½Í¨ï¿½Ã£ï¿½ï¿½ï¿½ï¿?
    TX_MSG.StdId = 0x280;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;

    // ï¿½ï¿½ï¿½ï¿½C620ï¿½ï¿½ï¿½Ğ?ï¿½ï¿½ï¿½ï¿½ï¿½Ãµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö?
    CAN_Send_Data[0] = c1;
    CAN_Send_Data[1] = (c1 >> 8);
    CAN_Send_Data[2] = c2;
    CAN_Send_Data[3] = (c2 >> 8);
    CAN_Send_Data[4] = c3;
    CAN_Send_Data[5] = (c3 >> 8);
    CAN_Send_Data[6] = c4;
    CAN_Send_Data[7] = (c4 >> 8);

    uint32_t count = 0;
    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    {
        count++;
        if (count > 10000)
            break;
    }
    count = 0;
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ä¶¼ï¿½ï¿½ï¿½ï¿½ï¿½Ë¾Íµï¿½Ò»ï¿½ï¿½ï¿½ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä³ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
    {
        count++;
        if (count > 10000)
            break;
    }
    /* Check Tx Mailbox 1 status */
    if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX0;
    }
    /* Check Tx Mailbox 1 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX1;
    }

    /* Check Tx Mailbox 2 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX2;
    }
    return HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

/**
  * @Func	    void Send_RMD_Current(CAN_HandleTypeDef* hcan,
                                    int16_t c1, int16_t c2, int16_t c3, int16_t c4)
  * @Brief
  * @Param	    cx x=1,2,3,4
  * @Retval	    None
  * @Date       2019/11/5
 **/
HAL_StatusTypeDef Send_LK_Current_Single(CAN_HandleTypeDef *_hcan,
                                         int8_t ID, int16_t c)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = 0x140 + ID;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;

    CAN_Send_Data[0] = 0xA1;
    CAN_Send_Data[1] = 0;
    CAN_Send_Data[2] = 0;
    CAN_Send_Data[3] = 0;
    CAN_Send_Data[4] = c;
    CAN_Send_Data[5] = (c >> 8);
    CAN_Send_Data[6] = 0;
    CAN_Send_Data[7] = 0;

    // uint32_t count = 0;
    // while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    // {
    //     count++;
    //     if (count > 10000)
    //         break;
    // }
    // count = 0;
    // while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0)
    // {
    //     count++;
    //     if (count > 10000)
    //         break;
    // }
    // /* Check Tx Mailbox 1 status */
    // if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
    // {
    //     send_mail_box = CAN_TX_MAILBOX0;
    // }
    // /* Check Tx Mailbox 1 status */
    // else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
    // {
    //     send_mail_box = CAN_TX_MAILBOX1;
    // }

    // /* Check Tx Mailbox 2 status */
    // else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
    // {
    //     send_mail_box = CAN_TX_MAILBOX2;
    // }
    return HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

int16_t LK8008_Current_Solver(float torque)
{
    return torque * LK8008_CURRENT_COEF;
}

/* -------------------------------- dm motor -------------------------------- */
uint8_t Data_Enable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};    // µç»úÊ¹ÄÜÃüÁî
uint8_t Data_Failure[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};   // µç»úÊ§ÄÜÃüÁî
uint8_t Data_Save_zero[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}; // µç»ú±£´æÁãµãÃüÁî

/**
 * @brief  ·¢ËÍ±ê×¼IDµÄÊı¾İÖ¡
 * @param  hcan     CANµÄ¾ä±ú
 * @param  ID       Êı¾İÖ¡ID
 * @param  pData    Êı×éÖ¸Õë
 * @param  Len      ×Ö½ÚÊı0~8
 */
uint8_t DM_CANx_SendStdData(CAN_HandleTypeDef *_hcan, uint16_t id, uint8_t cmd, uint16_t len)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    TX_MSG.StdId = id;
    TX_MSG.ExtId = 0;
    TX_MSG.IDE = 0;
    TX_MSG.RTR = 0;
    TX_MSG.DLC = len;

    if (cmd == ENABLE_MOTOR)
        memcpy(CAN_Send_Data, Data_Enable, len);
    else if (cmd == DISABLE_MOTOR)
        memcpy(CAN_Send_Data, Data_Failure, len);
    else if (cmd == SAVE_ZERO_MOTOR)
        memcpy(CAN_Send_Data, Data_Save_zero, len);
    else
        return 1; // ´íÎóµÄÃüÁî

    /*ÕÒµ½¿ÕµÄ·¢ËÍÓÊÏä£¬°ÑÊı¾İ·¢ËÍ³öÈ¥*/
    if (HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) //
    {
        if (HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
        {
            HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, (uint32_t *)CAN_TX_MAILBOX2);
        }
    }
    return 0;
}

/**
 * @brief  ²ÉÓÃ¸¡µãÊı¾İµÈ±ÈÀı×ª»»³ÉÕûÊı
 * @param  x_int     	Òª×ª»»µÄÎŞ·ûºÅÕûÊı
 * @param  x_min      Ä¿±ê¸¡µãÊıµÄ×îĞ¡Öµ
 * @param  x_max    	Ä¿±ê¸¡µãÊıµÄ×î´óÖµ
 * @param  bits      	ÎŞ·ûºÅÕûÊıµÄÎ»Êı
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    debugNum[0] = (float)x_int;
    debugNum[1] = (float)((1 << bits) - 1);
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

float uint_to_float1(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    debugNum[2] = (float)x_int;
    debugNum[3] = (float)((1 << bits) - 1);
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief  ½«¸¡µãÊı×ª»»ÎªÎŞ·ûºÅÕûÊı
 * @param  x     			Òª×ª»»µÄ¸¡µãÊı
 * @param  x_min      ¸¡µãÊıµÄ×îĞ¡Öµ
 * @param  x_max    	¸¡µãÊıµÄ×î´óÖµ
 * @param  bits      	ÎŞ·ûºÅÕûÊıµÄÎ»Êı
 */

int float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief  MITÄ£Ê½¿ØÏÂ¿ØÖÆÖ¡
 * @param  hcan   CANµÄ¾ä±ú
 * @param  ID     Êı¾İÖ¡µÄID
 * @param  _pos   Î»ÖÃ¸ø¶¨
 * @param  _vel   ËÙ¶È¸ø¶¨
 * @param  _KP    Î»ÖÃ±ÈÀıÏµÊı
 * @param  _KD    Î»ÖÃÎ¢·ÖÏµÊı
 * @param  _torq  ×ª¾Ø¸ø¶¨Öµ
 */
HAL_StatusTypeDef Send_DM_MIT_Command(CAN_HandleTypeDef *_hcan, uint16_t id, float _pos, float _vel,
                                      float _KP, float _KD, float _torq)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    TX_MSG.StdId = id;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;

    CAN_Send_Data[0] = (pos_tmp >> 8);
    CAN_Send_Data[1] = pos_tmp;
    CAN_Send_Data[2] = (vel_tmp >> 4);
    CAN_Send_Data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    CAN_Send_Data[4] = kp_tmp;
    CAN_Send_Data[5] = (kd_tmp >> 4);
    CAN_Send_Data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    CAN_Send_Data[7] = tor_tmp;

    // uint32_t count = 0;
    // while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    // {
    //     count++;
    //     if (count > 10000)
    //         break;
    // }
    // count = 0;
    // while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0)
    // {
    //     count++;
    //     if (count > 10000)
    //         break;
    // }
    // /* Check Tx Mailbox 1 status */
    // if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
    // {
    //     send_mail_box = CAN_TX_MAILBOX0;
    // }
    // /* Check Tx Mailbox 1 status */
    // else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
    // {
    //     send_mail_box = CAN_TX_MAILBOX1;
    // }

    // /* Check Tx Mailbox 2 status */
    // else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
    // {
    //     send_mail_box = CAN_TX_MAILBOX2;
    // }
    return HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Get_DM_Info(motor_info *ptr, uint8_t *aData)
{
    ptr->CAN_ID = aData[0] & 0xF;
    ptr->Err = aData[0] >> 4;
    ptr->dm_raw_angle = (aData[1] << 8) | aData[2];
    ptr->raw_velocity = (aData[3] << 4) | (aData[4] >> 4);
    ptr->raw_torque = ((aData[4] & 0xF) << 8) | aData[5];

    ptr->angle_in_degree = uint_to_float(ptr->dm_raw_angle, P_MIN, P_MAX, 16) * RADIAN_COEF; // (-12.5,12.5)
    ptr->velocity_in_radps = uint_to_float(ptr->raw_velocity, V_MIN, V_MAX, 12);             // (-45.0,45.0)
    ptr->torque_in_Nm = uint_to_float(ptr->raw_torque, T_MIN, T_MAX, 12);                    // (-18.0,18.0)

    ptr->Temperature = aData[7];
}

void Get_DM_Offset(motor_info *ptr, uint8_t *aData)
{
    ptr->dm_raw_angle = (aData[1] << 8) | aData[2];
    ptr->offset_angle = ptr->dm_raw_angle;
    ptr->last_angle = ptr->dm_raw_angle; // update last_angle
}
