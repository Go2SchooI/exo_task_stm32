#ifndef _SERIALDEBUG_H
#define _SERIALDEBUG_H

#include "usart.h"
#include "stdint.h"

extern uint8_t DebugBuffer[100];

void Serial_Debug(UART_HandleTypeDef *huart, uint16_t debug_period, float a, float b, float c, float d, float e, float f);
void Debug_Buf_Send(UART_HandleTypeDef *huart, uint8_t *data, uint8_t length);
void float2char(float floatdata, uint8_t *buffer, uint8_t n, uint8_t *position);
void Debug_Buf_Generate(UART_HandleTypeDef *huart, float a, float b, float c, float d, float e, float f, uint8_t *data);

#endif
