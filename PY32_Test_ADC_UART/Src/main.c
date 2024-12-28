/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandle;
GPIO_InitTypeDef GpioInitStruct;
GPIO_InitTypeDef AdcPinStruct;
ADC_HandleTypeDef AdcHandle;
ADC_ChannelConfTypeDef AdcChanConf;
TIM_HandleTypeDef tim1Handle;

uint32_t adcReading = 0;
char adcReadingStr[5];
char outputStr[100];


/* Private user code ---------------------------------------------------------*/
void Timer_config(void);
void Timer_tests(void);
void ADC_tests(void);
uint32_t Wait_until_tick(uint32_t tick, uint32_t max_wait);

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
void TIMER_Init(void);

/**
  * @brief  应用程序入口函数.
  * @retval int
  */
int main(void)
{
    uint32_t loop_count;
    
    HAL_Init();
  
    APP_SystemClockConfig(); 
	
	//Initilize needed periperhals
	GPIO_Init();
	TIMER_Init();
	UART_Init();
	ADC_Init();
    //Timer_config();
	
    for (loop_count = 0; loop_count < 10; loop_count++)
    {
		ADC_tests();
        //Timer_tests();		
		HAL_Delay(2000);
    }
    
    sprintf(outputStr, "End of tests.\r\n");		
	HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
    
    while (1);
}

uint32_t Wait_until_tick(uint32_t tick, uint32_t max_wait)
{
    uint32_t start_tick, current_tick, wait_time;
	start_tick = HAL_GetTick();
    if (start_tick >= tick) return 0;
    
    do {
        current_tick = HAL_GetTick();
        wait_time = current_tick - start_tick;
    } while (current_tick < tick  && wait_time < max_wait);
    return wait_time;
}

void Timer_config(void)
{
	uint32_t freq, freq_new;
    
	freq = HAL_GetTickFreq();
    HAL_SetTickFreq(1);     // set the tick timer to 1 MHz (1 msec per tick)
    
	freq_new = HAL_GetTickFreq();
    sprintf(outputStr, "timer tick freq: %d new: %d\r\n", freq, freq_new);		
	HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
}

void Timer_tests(void)
{
    uint32_t start_tick;
    uint32_t end_tick;
    uint32_t tick_wait_count;
    
	
	start_tick = HAL_GetTick();
	HAL_Delay(1);
	end_tick = HAL_GetTick();
	
    sprintf(outputStr, "HAL_Delay start tick: %d end_tick: %d\r\n", start_tick, end_tick);		
	HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	

  	HAL_Delay(100);

	start_tick = HAL_GetTick();
	tick_wait_count = Wait_until_tick(start_tick + 1, 10);
	end_tick = HAL_GetTick();
	
    sprintf(outputStr, "Wait_until_tick start tick: %d end_tick: %d tick_wait_count: %d\r\n", start_tick, end_tick, tick_wait_count);		
	HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
}

void ADC_tests(void)
{
    uint32_t start_tick;
    uint32_t end_tick;
    
	start_tick = HAL_GetTick();
    
    //Sample with ADC in polling mode
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle, 1000);
    adcReading = HAL_ADC_GetValue(&AdcHandle);

	end_tick = HAL_GetTick();
    
    //Compile string to send over UART
    sprintf(outputStr, "ADC %i #ticks: %d\r\n", adcReading, (end_tick-start_tick));
    
    //Send string over UART
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
}

void GPIO_Init(void)
{
	//Configure GPIO for ADC, UART
	//PA0 is TX
	//PA1 is RX
	
	//Enable clocks
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
	
	//Add setings to struct for UART
	GpioInitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GpioInitStruct.Mode = GPIO_MODE_AF_PP;
	GpioInitStruct.Pull = GPIO_PULLUP;
	GpioInitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GpioInitStruct.Alternate = GPIO_AF9_USART2;
	
	//Initialize
	HAL_GPIO_Init(GPIOA, &GpioInitStruct);
	
	
	//Pin settings for ADC input
	__HAL_RCC_ADC_CLK_ENABLE();
	
	AdcPinStruct.Pin = GPIO_PIN_2;
	AdcPinStruct.Mode = GPIO_MODE_ANALOG;
	AdcPinStruct.Pull = GPIO_NOPULL;
	
	//Initialize
	HAL_GPIO_Init(GPIOA, &AdcPinStruct);
    
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);    
}

void TIMER_Init(void)
{
	/* 	Trigger 4x per second, or every 250ms
			Clock at 24MHz -> 8 million cycles
			Period to 10,000, prescaler to 800
			10,000*800=8,000,000 */
	
	tim1Handle.Instance = TIM1;																						//Timer 1 advanced timer
    tim1Handle.Init.Period            = 15 - 1;				    //Timer count = (period+1)*(prescaler+1), Period of 30 = 1 msec and 15 = 500 usec
    tim1Handle.Init.Prescaler         = 800 - 1;
    tim1Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;						//Use full clock rate
    tim1Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    tim1Handle.Init.RepetitionCounter = 1 - 1;
    tim1Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&tim1Handle) != HAL_OK)
    {
    APP_ErrorHandler();
    }

    if (HAL_TIM_Base_Start_IT(&tim1Handle) != HAL_OK)
    {
    APP_ErrorHandler();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
}


void UART_Init(void)
{
	//Configure UART1
	//PA0 is TX
	//PA1 is RX
	
	UartHandle.Instance          = USART2;
  UartHandle.Init.BaudRate     = 115200;
  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
	
  if (HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

void ADC_Init(void)
{
	//Setup ADC to sample pin PA2 (channel 2)
	
	//Enable Clocks
	__HAL_RCC_ADC_FORCE_RESET();
  __HAL_RCC_ADC_RELEASE_RESET();
	__HAL_RCC_ADC_CLK_ENABLE();
	
	//Start calibration
	AdcHandle.Instance = ADC1;
	
	if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
		{
			APP_ErrorHandler();
		}
	
	//Populate ADC init data
	AdcHandle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_HSI_DIV1; //Full HSI speed
	AdcHandle.Init.Resolution = ADC_RESOLUTION_12B; //12 bits
	AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT; //Right aligned
	AdcHandle.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD; //Don't plan to use
	AdcHandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV; //single conversion
	AdcHandle.Init.LowPowerAutoWait = DISABLE; //use all the power
	AdcHandle.Init.ContinuousConvMode = DISABLE; //don't need for polling
	AdcHandle.Init.DiscontinuousConvMode = DISABLE; //don't need for polling
	AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START; //Will start ADC in code
	AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; //not going to use external trigger
	AdcHandle.Init.DMAContinuousRequests = DISABLE; //Not using DMA
	AdcHandle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN; //shouldn't matter for polling
	AdcHandle.Init.SamplingTimeCommon = ADC_SAMPLETIME_41CYCLES_5; //Setting conversion time to 41.5 cycles
		
	//Initialize ADC
	if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
	{
		APP_ErrorHandler();
	}
	
	//Now set ADC rank and channel
	AdcChanConf.Rank = 0; //highest rank, only using one channel
	AdcChanConf.Channel = ADC_CHANNEL_2; //for PA2
	
	if (HAL_ADC_ConfigChannel(&AdcHandle, &AdcChanConf) != HAL_OK)
	{
		APP_ErrorHandler();
	}
}

static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* 振荡器配置 */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI; /* 选择振荡器HSE,HSI,LSI */
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                          /* 开启HSI */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                          /* HSI 1分频 */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_24MHz;  /* Clock at 24MHz */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                         /* 关闭HSE */
  /*RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz;*/
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                         /* 关闭LSI */

  /* 配置振荡器 */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* 时钟源配置 */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1; /* 选择配置时钟 HCLK,SYSCLK,PCLK1 */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; /* 选择HSI作为系统时钟 */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;     /* AHB时钟 1分频 */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;      /* APB时钟 1分频 */
  /* 配置时钟源 */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

/**
  * @brief  错误执行函数
  * @param  无
  * @retval 无
  */
void APP_ErrorHandler(void)
{
  /* 无限循环 */
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  输出产生断言错误的源文件名及行号
  * @param  file：源文件名指针
  * @param  line：发生断言错误的行号
  * @retval 无
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* 用户可以根据需要添加自己的打印信息,
     例如: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* 无限循环 */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
