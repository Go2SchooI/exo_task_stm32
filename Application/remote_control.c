#include "remote_control.h"
#include "user_lib.h"
#include "bsp_dwt.h"

RC_Type remote_control;

uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

uint8_t RC_Data_Buffer[16] = {0};

static uint32_t DWT_RC_Count = 0;

void Remote_Control_Init(UART_HandleTypeDef *huart, uint8_t length)
{
    remote_control.RC_USART = huart;
    // ң��������ƽ���˲�����ʼ��
    // RC_Smooth_Filter_Init(filter_period, filter_num);
    // ң����������Ϣ��ʼ�� �����
    RC_USART_init(huart, sbus_rx_buf[0], sbus_rx_buf[1], length);
}

/**
 * @Func		void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
 * @Brief  	DR16���ջ�Э�������� ���DT7��ң�������ջ����ֲ�
 * @Param		RC_Type* rc���洢ң�������ݵĽṹ�塡��uint8_t* buff�����ڽ���Ļ���
 * @Retval		None
 * @Date
 */
void Callback_RC_Handle(RC_Type *rc, uint8_t *buff)
{
    if (buff == NULL || rc == NULL)
    {
        return;
    }
    memcpy(RC_Data_Buffer, buff, 16);

    rc->ch1_origin = (buff[0] | buff[1] << 8) & 0x07FF;
    rc->ch1_origin -= RC_CH_VALUE_OFFSET;
    rc->ch2_origin = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    rc->ch2_origin -= RC_CH_VALUE_OFFSET;
    rc->ch3_origin = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc->ch3_origin -= RC_CH_VALUE_OFFSET;
    rc->ch4_origin = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
    rc->ch4_origin -= RC_CH_VALUE_OFFSET;

    rc->switch_left = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->switch_right = (buff[5] >> 4) & 0x0003;

    rc->mouse.x_origin = buff[6] | (buff[7] << 8); // x axis
    rc->mouse.y_origin = buff[8] | (buff[9] << 8);
    rc->mouse.z_origin = buff[10] | (buff[11] << 8);

    rc->mouse.press_left = buff[12]; // is pressed?
    rc->mouse.press_right = buff[13];

    rc->key_code = buff[14] | buff[15] << 8; // key borad code

    if (rc->switch_left != 1 && rc->switch_left != 2 && rc->switch_left != 3)
    {
        rc->ch1_origin = 0;
        rc->ch2_origin = 0;
        rc->ch3_origin = 0;
        rc->ch4_origin = 0;
        rc->switch_left = 0;
        rc->switch_right = 0;

        rc->mouse.x_origin = 0; // x axis
        rc->mouse.y_origin = 0;
        rc->mouse.z_origin = 0;

        rc->mouse.press_left = 0; // is pressed?
        rc->mouse.press_right = 0;

        rc->key_code = 0; // key borad code
    }
}

void RC_USART_init(UART_HandleTypeDef *huart, uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // enable the DMA transfer for the receiver request
    // ʹ��DMA���ڽ���
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    // enalbe idle interrupt
    // ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    // disable DMA
    // ʧЧDMA
    __HAL_DMA_DISABLE(huart->hdmarx);
    while (huart->hdmarx->Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(huart->hdmarx);
    }

    huart->hdmarx->Instance->PAR = (uint32_t)&(USART1->DR);
    // memory buffer 1
    // �ڴ滺����1
    huart->hdmarx->Instance->M0AR = (uint32_t)(rx1_buf);
    // memory buffer 2
    // �ڴ滺����2
    huart->hdmarx->Instance->M1AR = (uint32_t)(rx2_buf);
    // data length
    // ���ݳ���
    huart->hdmarx->Instance->NDTR = dma_buf_num;
    // enable double memory buffer
    // ʹ��˫������
    SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);

    // enable DMA
    // ʹ��DMA
    __HAL_DMA_ENABLE(huart->hdmarx);
}

void RC_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    // �����
    uint32_t isrflags = READ_REG(huart->Instance->SR);
    uint32_t cr1its = READ_REG(huart->Instance->CR1);
    uint32_t cr3its = READ_REG(huart->Instance->CR3);

    if (huart->Instance->SR & UART_FLAG_RXNE) // ���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(huart);
    }
    else if (huart->Instance->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(huart);
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        if ((huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET) // CT => Current target
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            // ʧЧDMA
            __HAL_DMA_DISABLE(huart->hdmarx);

            // get receive data length, length = set_data_length - remain_length
            // ��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - huart->hdmarx->Instance->NDTR;

            // reset set_data_lenght
            // �����趨���ݳ���
            huart->hdmarx->Instance->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 1
            // �趨������1
            huart->hdmarx->Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            // ʹ��DMA
            __HAL_DMA_ENABLE(huart->hdmarx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                Callback_RC_Handle(&remote_control, sbus_rx_buf[0]);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            // ʧЧDMA
            __HAL_DMA_DISABLE(huart->hdmarx);

            // get receive data length, length = set_data_length - remain_length
            // ��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - huart->hdmarx->Instance->NDTR;

            // reset set_data_lenght
            // �����趨���ݳ���
            huart->hdmarx->Instance->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 0
            // �趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // ʹ��DMA
            __HAL_DMA_ENABLE(huart->hdmarx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                // ����ң��������
                Callback_RC_Handle(&remote_control, sbus_rx_buf[1]);
            }
        }
    }

    // �öδ�������������޹أ����ڱ�֤DMA������������
    if (((isrflags & USART_SR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
    {
        /* Disable the UART Transmit Complete Interrupt */
        __HAL_UART_DISABLE_IT(huart, UART_IT_TC);

        /* Tx process is ended, restore huart->gState to Ready */
        huart->gState = HAL_UART_STATE_READY;

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        /*Call registered Tx complete callback*/
        huart->TxCpltCallback(huart);
#else
        /*Call legacy weak Tx complete callback*/
        HAL_UART_TxCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
    }
}

void HAL_USART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == remote_control.RC_USART)
        RC_USART_init(remote_control.RC_USART, sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
    // else
    //     AutoAim_USART_Init(huart, AutoAim_RX_Buf[0], AutoAim_RX_Buf[1], AutoAim_RX_BUF_NUM);
}
