#ifndef _BSP_CAN_H
#define _BSP_CAN_H

#include "includes.h"

#define LK_DEVICE_STD_ID (0x140)
#define LK_DEVICE_STD_BOARDCAST_ID (0x280)

#define DM_DEVICE_STD_ID (0x10)

// void CAN_Device_Init(CAN_HandleTypeDef *_hcan);
void CAN_Device_Init(void);

#endif
