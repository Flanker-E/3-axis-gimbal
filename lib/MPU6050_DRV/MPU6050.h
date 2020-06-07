#ifndef _I2C_USER_DEFINE_H
#define _I2C_USER_DEFINE_H

#include "stm32f1xx_hal.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

extern I2C_HandleTypeDef* MPU6050_I2C;

HAL_StatusTypeDef init_MPU6050(I2C_HandleTypeDef* hi2c);
HAL_StatusTypeDef init_MPU6050_DMP(I2C_HandleTypeDef* hi2c);

#endif