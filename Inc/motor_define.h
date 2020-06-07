#ifndef _MOTOR_DEFINE_H
#define _MOTOR_DEFINE_H

#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;

#define MOTOR_POLE_PAIRS 6
#define MOTOR_SPEED_TIMER_FRQ 1000000

#define MOTOR0_TIM &htim3
#define MOTOR0_PHASE_A TIM_CHANNEL_2
#define MOTOR0_PHASE_B TIM_CHANNEL_3
#define MOTOR0_PHASE_C TIM_CHANNEL_4
#define MOTOR0_SPEED_TIM &htim2

#endif