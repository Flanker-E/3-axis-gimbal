#ifndef _SERIAL_PRINT_H
#define _SERIAL_PRINT_H

#include "stm32f1xx_hal.h"
//#include "as5600.h"
#include "sine_wave.h"
#pragma pack(1)
typedef struct
{
    uint32_t time_stamp;
    float yaw;
    float roll;
    float pitch;
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

typedef struct 
{
    UART_HandleTypeDef  *uart;
    uint8_t is_compelete;
    uint16_t    rx_bytes_count;
    Serial_Receive_Stream_typeDef   data;
    HAL_StatusTypeDef *rx_callback;
    uint8_t tmp_buff;
}Serial_Receive_IT_typeDef;

extern  Serial_Receive_IT_typeDef   uart3_it;

HAL_StatusTypeDef SerialPrintTransmit(Serial_Transmit_Stream_typeDef *data);
HAL_StatusTypeDef  InitSerialPrintReceiveIT(Serial_Receive_IT_typeDef *it,UART_HandleTypeDef *uart);
HAL_StatusTypeDef SerialPrintReceive(const Serial_Receive_Stream_typeDef *data);

#endif