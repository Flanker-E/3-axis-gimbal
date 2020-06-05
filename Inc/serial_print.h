#ifndef _SERIAL_PRINT_H
#define _SERIAL_PRINT_H

#include "stm32f1xx_hal.h"

#pragma pack(1)
typedef struct
{
//    uint32_t time_stamp;
//    float yaw;
//    float roll;
//    float pitch;
    float test_float;
    char end1;
    char end2;
} Serial_Transmit_Stream_typeDef;

typedef struct
{
    uint32_t time_stamp;
    float yaw;
    float roll;
    float pitch;
    char end1;
    char end2;
} Serial_Receive_Stream_typeDef;
#pragma pack()

extern UART_HandleTypeDef huart3;

HAL_StatusTypeDef SerialPrintTransmit(Serial_Transmit_Stream_typeDef *data);
HAL_StatusTypeDef SerialPrintReceive(const Serial_Receive_Stream_typeDef *data);

#endif