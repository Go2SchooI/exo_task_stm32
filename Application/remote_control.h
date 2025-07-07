#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"

#define RCFILTER_TASK_PERIOD 1

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

#define Key_W ((uint16_t)0x01 << 0)
#define Key_S ((uint16_t)0x01 << 1)
#define Key_A ((uint16_t)0x01 << 2)
#define Key_D ((uint16_t)0x01 << 3)
#define Key_SHIFT ((uint16_t)0x01 << 4)
#define Key_CTRL ((uint16_t)0x01 << 5)
#define Key_Q ((uint16_t)0x01 << 6)
#define Key_E ((uint16_t)0x01 << 7)
#define Key_R ((uint16_t)0x01 << 8)
#define Key_F ((uint16_t)0x01 << 9)
#define Key_G ((uint16_t)0x01 << 10)
#define Key_Z ((uint16_t)0x01 << 11)
#define Key_X ((uint16_t)0x01 << 12)
#define Key_C ((uint16_t)0x01 << 13)
#define Key_V ((uint16_t)0x01 << 14)
#define Key_B ((uint16_t)0x01 << 15)

typedef struct
{
    UART_HandleTypeDef *RC_USART;

    int16_t ch1_origin;
    int16_t ch2_origin;
    int16_t ch3_origin;
    int16_t ch4_origin;

    // value after filter
    float ch1;
    float ch2;
    float ch3;
    float ch4;
    float ch1_dot;
    float ch1_ddot;

    uint8_t switch_left; // 3 value
    uint8_t switch_right;

    struct
    {
        int16_t x_origin;
        int16_t y_origin;
        int16_t z_origin;

        float x;
        float y;
        float z;

        uint8_t press_left;
        uint8_t press_right;
    } mouse;

    uint16_t key_code;
    /**********************************************************************************
     * ����ͨ��: 15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
     *           V    C    X	   Z    G    F   R   E   Q  CTRL  SHIFT  D   A   S   W
     ************************************************************************************/

    uint8_t RC_state;

} RC_Type;

enum
{
    Switch_Up = 1,
    Switch_Middle = 3,
    Switch_Down = 2,
};

enum
{
    Disconnected = 0,
    Connected = 1,
};

extern RC_Type remote_control;

extern uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
extern uint8_t RC_Data_Buffer[16];

extern uint64_t Latest_Remote_Control_Pack_Time;

void Remote_Control_Init(UART_HandleTypeDef *huart, uint8_t length);
void Callback_RC_Handle(RC_Type *rc, uint8_t *buff);
void RC_Smooth_Filter_Init(float period, float num);
void RC_Smooth_Filter(void);
void Solve_RC_Lost(void);
void Solve_RC_Data_Error(void);
uint8_t RC_Data_is_Error(void);

void RC_USART_init(UART_HandleTypeDef *huart, uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void RC_UART_IRQHandler(UART_HandleTypeDef *huart);
void RC_Restart(uint16_t dma_buf_num);

#endif
