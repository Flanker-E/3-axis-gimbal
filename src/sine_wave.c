#include "sine_wave.h"
#include "math.h"
#include "tim.h"

uint16_t sineWaveTable[SINE_RANGE] = {0};

void InitSineWaveTable() //计算sin值用于查表
{
    for (uint16_t i = 0; i < SINE_RANGE; i++) 
    {
        float x = i * M_TWOPI / SINE_RANGE;
        sineWaveTable[i] = (uint16_t)((sinf(x) + 1) / 2 * VMAX_SCALE); //归一到[0,1]内，再归一到最高占空比内
    }
}

void setMotorPWM(float angle, float Vmax)
{
    float Magnitude = Vmax / VMAX_OUTPUT; //取相对于可输出最大电压的比值
    /*取三相对应正弦表的索引*/
    uint16_t angleA = (((uint16_t)(angle * SINE_RANGE / M_TWOPI)) % SINE_RANGE);
    uint16_t angleB = (((uint16_t)((angle + M_TWOPI / 3) * SINE_RANGE / M_TWOPI)+1) % SINE_RANGE);
    uint16_t angleC = (((uint16_t)((angle + 2 * M_TWOPI / 3) * SINE_RANGE / M_TWOPI+1)) % SINE_RANGE);

    /*设置三通道对应的占空比*/
    setPhasePWM(&htim3, TIM_CHANNEL_2, (uint16_t)(Magnitude * sineWaveTable[angleA]));
    setPhasePWM(&htim3, TIM_CHANNEL_3, (uint16_t)(Magnitude * sineWaveTable[angleB]));
    setPhasePWM(&htim3, TIM_CHANNEL_4, (uint16_t)(Magnitude * sineWaveTable[angleC]));
}

void setPhasePWM(TIM_HandleTypeDef *tim, uint32_t channel, uint16_t count) //将正弦表取值分发到各channel的CCRx寄存器中
{
    if (count < 0)
        count = 0;
    if (count > VMAX_SCALE*2)
        count = VMAX_SCALE*2;

    switch (channel)
    {
    case TIM_CHANNEL_2:
        tim->Instance->CCR2 = count;
        break;
    case TIM_CHANNEL_3:
        tim->Instance->CCR3 = count;
        break;
    case TIM_CHANNEL_4:
        tim->Instance->CCR4 = count;
        break;
    default:
        break;
    }
}