#include "MPU6050.h"
#include "i2c_user_interface.h"

I2C_HandleTypeDef *MPU6050_I2C;

int stm32_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                    unsigned char length, unsigned char const *data)
{
    return HAL_I2C_Mem_Write(MPU6050_I2C, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, length, 0xfff);
}

int stm32_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                   unsigned char length, unsigned char *data)
{
    return HAL_I2C_Mem_Read(MPU6050_I2C, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, length, 0xfff);
}