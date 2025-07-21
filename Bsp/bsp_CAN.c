#include "bsp_CAN.h"
#include "exo_controller.h"

int16_t RM_CAN_Msg_0x200[4]; // ID 1~4
int16_t RM_CAN_Msg_0x1FF[4]; // ID 5~8
int16_t RM_CAN_Msg_0x2FF[3]; // ID 9~11

uint8_t rdata[8]; // CAN receive data buffer

/**
 * @Func		CAN_Device_Init
 * @Brief
 * @Param		CAN_HandleTypeDef* hcan
 * @Retval		None
 * @Date       2019/11/4
 **/
void CAN_Device_Init(void)
{
    // ��ʼ��CAN������Ϊ������״̬ ��Ϊ�������� �����
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    // CAN ��������ʼ��
    while (HAL_CAN_ConfigFilter(&hcan1, &can_filter_st) != HAL_OK)
    {
    }
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    // ����CAN
    while (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
    }
    HAL_CAN_Start(&hcan1);
    // ����֪ͨ
    while (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
    }
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    // CAN ��������ʼ��
    while (HAL_CAN_ConfigFilter(&hcan2, &can_filter_st) != HAL_OK)
    {
    }
    // ����CAN
    while (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
    }
    // ����֪ͨ
    while (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
    }
}

/**
 * @Func	    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* _hcan)
 * @Brief       This function is called when a message is pending in the Rx FIFO 0.
 * @Param	    CAN_HandleTypeDef* _hcan
 * @Retval	    None
 * @Date       2019/11/4
 **/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *_hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    memcpy(rdata, rx_data, 8);

    if (_hcan == &hcan1)
    {
        switch (rx_header.StdId)
        {
        // case LK_DEVICE_STD_ID + 1:
        //     if (exo_controller.xzy_shoulder.lk_motor.msg_cnt++ <= 50)
        //         Get_LK_Offset(&exo_controller.xzy_shoulder.lk_motor, rx_data);
        //     else
        //         Get_LK_Info(&exo_controller.xzy_shoulder.lk_motor, rx_data);
        //     break;
        case DM_DEVICE_STD_ID + 1:
            if (exo_controller.xzy_shoulder.dm_motor[0].msg_cnt++ <= 50)
                Get_DM_Offset(&exo_controller.xzy_shoulder.dm_motor[0], rx_data);
            else
                Get_DM_Info(&exo_controller.xzy_shoulder.dm_motor[0], rx_data);
            break;
        case DM_DEVICE_STD_ID + 3:
            if (exo_controller.xzy_shoulder.dm_motor[1].msg_cnt++ <= 50)
                Get_DM_Offset(&exo_controller.xzy_shoulder.dm_motor[1], rx_data);
            else
                Get_DM_Info(&exo_controller.xzy_shoulder.dm_motor[1], rx_data);
            break;
        case DM_DEVICE_STD_ID + 2:
            if (exo_controller.elbow.dm_motor.msg_cnt++ <= 50)
                Get_DM_Offset(&exo_controller.elbow.dm_motor, rx_data);
            else
                Get_DM_Info(&exo_controller.elbow.dm_motor, rx_data);
            break;
        }
    }

    if (_hcan == &hcan2)
    {
        switch (rx_header.StdId)
        {
        case LK_DEVICE_STD_ID + 1:
            if (exo_controller.xzy_shoulder.lk_motor.msg_cnt++ <= 50)
                Get_LK_Offset(&exo_controller.xzy_shoulder.lk_motor, rx_data);
            else
                Get_LK_Info(&exo_controller.xzy_shoulder.lk_motor, rx_data);
            break;
        case 0x11:
            IMU_UpdateData(&exo_controller.xzy_shoulder.dm_imu, rx_data);
        }
    }
}
