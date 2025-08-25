#ifndef __INCLUDES_H
#define __INCLUDES_H

#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif

// #ifndef __CC_ARM
// #define __CC_ARM
// #endif

#ifndef ARM_MATH_MATRIX_CHECK
#define ARM_MATH_MATRIX_CHECK
#endif

#ifndef ARM_MATH_ROUNDING
#define ARM_MATH_ROUNDING
#endif

#ifndef ARM_MATH_DSP
#define ARM_MATH_DSP
#endif

// CubeMX
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
// #include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

// CubeMX
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
// #include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

// std C
#include <stdio.h>
#include <stdint.h>
#include <math.h>

// user lib
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"

// bsp
#include "bsp_CAN.h"
#include "bsp_dwt.h"
#include "bsp_PWM.h"

// application
#include "motor.h"
#include "remote_control.h"
#include "SerialDebug.h"
#include "ins_task.h"
// #include "exo_controller.h"

// DSP lib
#include "arm_math.h"

#endif
