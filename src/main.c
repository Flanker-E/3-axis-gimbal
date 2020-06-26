/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "serial_server.h"
#include "sine_wave.h"
#include "motor_define.h"
#include "SVPWM.h"
#include "MPU6050.h"
#include "stm32f1xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
Serial_Transmit_Stream_typeDef serial_data;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ADC2_Value = 0;
static float P=90.0;
static float I=0.0;
static float D=100.0;
static uint8_t motor_num=0;
static float pitchPPara = 120.0;
static float pitchIPara = 0.0;
static float pitchDPara = 10.0;
static float rollPPara = 120.0;
static float rollIPara = 0.0;
static float rollDPara = 10.0;
static float yawPPara = 80.0;
static float yawIPara = 0.0;
static float yawDPara = 8.0;
//__IO uint32_t currentTicks = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// P=90;
// I=0;
// D=100;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // float angle = 0;
  // float motor_v = 2.5;
  float yaw, roll, pitch;
  
  uint8_t delay_count=0;
  uint32_t delay_time=0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_ADC2_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  InitSerialPrintReceiveIT(&uart3_it,&huart3);
  HAL_UART_Receive_IT(uart3_it.uart,&(uart3_it.tmp_buff),1);
  // initiate client's PID value
  uint8_t n=0;
  while(n<3)
  {
    if (n==0)
    {
      serial_data.time_stamp = ((uint32_t)0);
      P=pitchPPara;
      I=pitchIPara;
      D=pitchDPara;
    }
    else if (n==1)
    {
      serial_data.time_stamp = ((uint32_t)1);
      P=rollPPara;
      I=rollIPara;
      D=rollDPara;
    }
    else
    {
      serial_data.time_stamp = ((uint32_t)2);
      P=yawPPara;
      I=yawIPara;
      D=yawDPara;
    }
    serial_data.end1 = '\r';
    serial_data.end2 = '\n';
    serial_data.yaw = (float)P;
    serial_data.roll = (float)I;
    serial_data.pitch = (float)D;
    for(int i=0;i<2;i++)
    {
      SerialPrintTransmit(&serial_data);
      HAL_Delay(50);
    }
    n++;
  }

  /* Number of ticks per millisecond */
  const uint32_t tickPerMs = SysTick->LOAD + 1;
  // HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  // HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
  // HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

   //PB12 Led0 green
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,SET);
  HAL_Delay(2000);
  //Set is pull-up
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,RESET);
  HAL_Delay(500);
  //Reset is pull-down

  /*Onboard_MPU6050*/
  // is_using_onboard_mpu=1;
  // /*global variable which indicates the in use mpu*/
  // /*when set to 1,the address whould be plused to reach the other mpu*/
  // while (init_MPU6050_DMP(&hi2c1))
  // {
  //   printf("Onboard_MPU6050 DMP init error!\r\n");
  //   HAL_Delay(50);
  // }
  /*External_MPU6050*/
  is_using_onboard_mpu=0;
  // while (init_MPU6050_DMP(&hi2c2))
  // {
  //   printf("External_MPU6050 DMP init error!\r\n");
  //   HAL_Delay(50);
  // }
  

 
  //added in the end of data
  //serial_data.test_float=123.0;
  //serial_data.time_stamp = HAL_GetTick();//get current time
  // SerialPrintTransmit(&serial_data);
  // HAL_Delay(200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {
    if(uart3_it.is_compelete){
      uart3_it.is_compelete = 0;
      // updateEncoder(&roll_encoder);
      motor_num= uart3_it.data.time_stamp;
      if(sendpid(motor_num,&uart3_it.data.yaw,&uart3_it.data.roll,&uart3_it.data.pitch))
      {
        serial_data.time_stamp = uart3_it.data.time_stamp;
        serial_data.yaw = uart3_it.data.yaw;
        serial_data.roll = uart3_it.data.roll;
        serial_data.pitch = uart3_it.data.pitch;
        SerialPrintTransmit(&serial_data);
        printf("transmited_to_client\r\n");
      }
      printf("transmited_to_client\r\n");
      HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
    }
    //PB12 Led0 green
    // HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,SET);
    // HAL_Delay(200);
    // //Set is pull-up
    // HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,RESET);
    // HAL_Delay(20);
    //Reset is pull-down

    if(frame_1Hz==1)//toggle green and send temp. each second
    { 
      frame_1Hz=0;
      HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
      HAL_ADC_Start(&hadc2);//Activate adc conversion. If isn't executed, adc can't get new value
      HAL_ADC_PollForConversion(&hadc2, 50);
      /* Check if the continous conversion of regular channel is finished */  
      if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2), HAL_ADC_STATE_REG_EOC)) 
      {
        /*##-3- Get the converted value of regular channel  ######################*/
        ADC2_Value = HAL_ADC_GetValue(&hadc2);
        //printf("Voltage: %1.3f V \r\n",ADC2_Value*0.0061767578125); 
        /*voltage coefficient?0.00617=3.3/4096*11.5/1.5?comes from divided voltage compared to VCC*/
      }
    }

    if(frame_500Hz==1)
    {
      frame_500Hz=0;

      __IO uint32_t currentTicks = SysTick->VAL;
      //currentTicks       = SysTick->VAL;//get current time
      //deltaTime500Hz    = currentTicks - previous500HzTime;//time between loop
      previous500HzTime = currentTicks;//record time before functions

      // is_using_onboard_mpu=0;
      // MPU6050_I2C = &hi2c2;
      // while (mpu_dmp_get_data(&pitch, &roll, &yaw));//get data from mpu#2
      // if(delay_count!=0)//ignore printf time
      // {
      // delay_time+=executionTime500Hz;
      // }
      // delay_count++;
      // if(delay_count==101)
      // {
      //   delay_count=0;
      //   printf("------------External_MPU6050-------------\r\n");
      //   printf("yaw:%.2f\troll:%.2f\tpitch:%.2f\r\n", yaw, roll, pitch);
      //   printf("consume time:%.2f\r\n",((float)delay_time)/100);
      //   printf("tickperms:%d\r\n",tickPerMs);
      //   delay_time=0;
      // }
      // // is_using_onboard_mpu=1;
      // MPU6050_I2C = &hi2c1;
      // while (mpu_dmp_get_data(&pitch, &roll, &yaw));//get data from mpu#1
      // printf("------------Onboard_MPU6050-------------\r\n");
      // printf("yaw:%.2f\troll:%.2f\tpitch:%.2f\r\n", yaw, roll, pitch);
      // printf("--------------------------------\r\n");
      // serial_data.pitch=pitch;
      // serial_data.roll=roll;
      // serial_data.yaw=yaw;
      // serial_data.time_stamp = HAL_GetTick();//get current time
      // SerialPrintTransmit(&serial_data);
      // HAL_Delay(200);
      currentTicks = SysTick->VAL;
      executionTime500Hz = (previous500HzTime < currentTicks) ? tickPerMs + previous500HzTime - currentTicks :
                    previous500HzTime - currentTicks;//save this execute time to executionTime500Hz
    }

    if(frame_100Hz==1)
    {
      frame_100Hz=0;
      HAL_UART_Receive_IT(uart3_it.uart,&(uart3_it.tmp_buff),1);
      //HAL_Delay(1000);
    }
    
    
    // printf("12345\r\n");

    //PB13 Led1 red
    // HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,SET);
    // HAL_Delay(200);
    // HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,RESET);
    // HAL_Delay(20);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int sendpid(int motornum,float *get_P, float *get_I, float *get_D)
{
  static int PID_transmit_complete=0;
  PID_transmit_complete=0;
  if(motornum==0)
  {
      pitchPPara=*get_P;
      pitchIPara=*get_I;
      pitchDPara=*get_D;
      printf("storm_pitch:P:%f,I:%f,D:%f\r\n",pitchPPara,pitchIPara,pitchDPara);
  }
  else if(motornum==1)
  {
      rollPPara=*get_P;
      rollIPara=*get_I;
      rollDPara=*get_D;
      printf("storm_roll:P:%f,I:%f,D:%f\r\n",rollPPara,rollIPara,rollDPara);
  }
  else
  {
      yawPPara=*get_P;
      yawIPara=*get_I;
      yawDPara=*get_D;
      printf("storm_yaw:P:%f,I:%f,D:%f\r\n",yawPPara,yawIPara,yawDPara);
  }
  
  PID_transmit_complete=1;
  return PID_transmit_complete;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
