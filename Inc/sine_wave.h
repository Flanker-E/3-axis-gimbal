#ifndef _SINE_WAVE_H
#define _SINE_WAVE_H

#include "stm32f1xx_hal.h"

#define VMAX_INPUT 12.0f
#define VMAX_SCALE 10923 //最高占空比相对于2^16的比值
#define SINE_RANGE 1024
#define M_TWOPI 6.28318531f //精度较高的2*pi

#define VMAX_OUTPUT (VMAX_INPUT * VMAX_SCALE / 0xFFFF)

extern uint16_t sineWaveTable[SINE_RANGE];

void InitSineWaveTable();
void setMotorPWM(float angle, float Vmax);
void setPhasePWM(TIM_HandleTypeDef *tim, uint32_t channel, uint16_t count);
#endif