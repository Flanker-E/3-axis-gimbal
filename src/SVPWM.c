#include "SVPWM.h"

int8_t dir = 0;
uint16_t idx = 0;

void setMotorSVPWM(TIM_HandleTypeDef *tim, uint32_t channel, uint16_t count, float voltage)
{
    float kv = BOUND(voltage, 0.0, VOLTAGE_MAX) / VOLTAGE_INPUT;
    count = (uint16_t)(kv * count);

    count = BOUND(count, 0, VOLTAGE_MAX * COUNT_MAX / VOLTAGE_INPUT);
    switch (channel)
    {
    case TIM_CHANNEL_1:
        tim->Instance->CCR1 = count;
        break;
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

void setAngleSVPWM(TIM_HandleTypeDef *motor, float angle, float voltage)
{
    uint16_t index = (((uint32_t)(angle * SVPWM_TABLE_SIZE / M_TWOPI)) % SVPWM_TABLE_SIZE);

    setMotorSVPWM(motor, MOTOR0_PHASE_A, u16_svpwm_table_A[index], voltage);
    setMotorSVPWM(motor, MOTOR0_PHASE_B, u16_svpwm_table_B[index], voltage);
    setMotorSVPWM(motor, MOTOR0_PHASE_C, u16_svpwm_table_C[index], voltage);
}

void setSpeedSVPWM(TIM_HandleTypeDef *motor_speed_tim, float speed, float voltage)
{
    uint16_t arr = 0xFFFF;
    if (speed == 0)
    {
        __HAL_TIM_SET_AUTORELOAD(motor_speed_tim, arr);
    }
    else
    {
        dir = speed > 0 ? 1 : -1;
        speed = speed > 0 ? speed : -speed;
        arr = (M_TWOPI * MOTOR_SPEED_TIMER_FRQ) / (SVPWM_TABLE_SIZE * speed * MOTOR_POLE_PAIRS);
        __HAL_TIM_SET_AUTORELOAD(motor_speed_tim, arr);
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim == MOTOR0_SPEED_TIM){
        idx= (idx+dir+SVPWM_TABLE_SIZE)%SVPWM_TABLE_SIZE;
        setMotorSVPWM(MOTOR0_TIM, MOTOR0_PHASE_A, u16_svpwm_table_A[idx], VOLTAGE_MAX/2);
        setMotorSVPWM(MOTOR0_TIM, MOTOR0_PHASE_B, u16_svpwm_table_B[idx], VOLTAGE_MAX/2);
        setMotorSVPWM(MOTOR0_TIM, MOTOR0_PHASE_C, u16_svpwm_table_C[idx], VOLTAGE_MAX/2);
        HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
    }
}