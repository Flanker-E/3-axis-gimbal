#include"MPU6050.h"

HAL_StatusTypeDef init_MPU6050(I2C_HandleTypeDef* hi2c){
    MPU6050_I2C = hi2c;
    if(mpu_init())  return HAL_ERROR;
    return  HAL_OK;
}

HAL_StatusTypeDef init_MPU6050_DMP(I2C_HandleTypeDef* hi2c){
    MPU6050_I2C = hi2c;
    if(mpu_dmp_init())  return HAL_ERROR;
    return HAL_OK;
}