#include "serial_print.h"
#include "SVPWM.h"
#include "motor_define.h"
#include "memory.h"
// extern AS5600_TypeDef roll_encoder;
Serial_Receive_IT_typeDef   uart3_it;
extern Serial_Transmit_Stream_typeDef serial_data;

HAL_StatusTypeDef SerialPrintTransmit(Serial_Transmit_Stream_typeDef *data)
{
    return HAL_UART_Transmit(&huart3, (uint8_t *)data, sizeof(Serial_Transmit_Stream_typeDef), 0xffff);
}

HAL_StatusTypeDef InitSerialPrintReceiveIT(Serial_Receive_IT_typeDef *it, UART_HandleTypeDef *uart)
{
    it->uart = uart;
    it->rx_bytes_count = 0;
    it->is_compelete = 0;
    return HAL_OK;
}

HAL_StatusTypeDef SerialPrintReceive(const Serial_Receive_Stream_typeDef *data)
{
    return HAL_UART_Transmit(&huart3, (uint8_t *)data, sizeof(Serial_Transmit_Stream_typeDef), 0xffff);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (uart3_it.rx_bytes_count >= 18)
        {
            uart3_it.rx_bytes_count = 0;
            memset(&(uart3_it.data), 0, sizeof(Serial_Receive_Stream_typeDef));
            uart3_it.is_compelete = 0;
        }
        else
        {
            uint8_t *ptr = &(uart3_it.data);
            ptr[uart3_it.rx_bytes_count++] = uart3_it.tmp_buff;
            if (ptr[uart3_it.rx_bytes_count - 2] == '\r' && ptr[uart3_it.rx_bytes_count - 1] == '\n')
            {
                uart3_it.is_compelete = 1;
                // serial_data.yaw = roll_encoder.value;
                serial_data.roll = uart3_it.data.roll*9.55;
                serial_data.pitch = uart3_it.data.yaw*10.0;
                SerialPrintTransmit(&serial_data);
                setAngleSVPWM(MOTOR0_TIM,uart3_it.data.roll, uart3_it.data.yaw);
                HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
                uart3_it.rx_bytes_count = 0;
            }else
            {
                uart3_it.is_compelete = 0;
            }
        }
        HAL_UART_Receive_IT(uart3_it.uart,&(uart3_it.tmp_buff),1);        
    }
}
