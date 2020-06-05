#include "serial_print.h"

HAL_StatusTypeDef   SerialPrintTransmit(Serial_Transmit_Stream_typeDef *data){
    return HAL_UART_Transmit(&huart3,(uint8_t*)data,sizeof(Serial_Transmit_Stream_typeDef),0xffff);
}

HAL_StatusTypeDef   SerialPrintReceive(const Serial_Receive_Stream_typeDef *data){

}