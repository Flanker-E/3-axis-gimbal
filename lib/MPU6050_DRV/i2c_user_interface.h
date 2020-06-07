#ifndef _I2C_USER_INTERFACE_H
#define _I2C_USER_INTERFACE_H

// i2c_write(unsigned char slave_addr, unsigned char reg_addr,
//      unsigned char length, unsigned char const *data)
int stm32_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                    unsigned char length, unsigned char const *data);

// i2c_read(unsigned char slave_addr, unsigned char reg_addr,
//      unsigned char length, unsigned char *data)
int stm32_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                   unsigned char length, unsigned char *data);


#endif