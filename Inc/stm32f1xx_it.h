/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
 ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F1xx_IT_H
#define __STM32F1xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint16_t frameCounter ;
extern uint8_t frame_500Hz ;
extern uint8_t frame_100Hz ;
extern uint8_t frame_50Hz  ;
extern uint8_t frame_10Hz  ;
extern uint8_t frame_5Hz   ;
extern uint8_t frame_1Hz   ; 

extern uint32_t deltaTime1000Hz, executionTime1000Hz, previous1000HzTime;
extern uint32_t deltaTime500Hz,  executionTime500Hz,  previous500HzTime;
extern uint32_t deltaTime100Hz,  executionTime100Hz,  previous100HzTime;
extern uint32_t deltaTime50Hz,   executionTime50Hz,   previous50HzTime;
extern uint32_t deltaTime10Hz,   executionTime10Hz,   previous10HzTime;
extern uint32_t deltaTime5Hz,    executionTime5Hz,    previous5HzTime;
extern uint32_t deltaTime1Hz,    executionTime1Hz,    previous1HzTime;

extern uint32_t currentTime;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
