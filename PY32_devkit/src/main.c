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

uint32_t adcReading[5] = {0, 0, 0, 0, 0};
char adcReadingStr[5];
char outputStr[100];

uint32_t TIM1_cb_count;
uint32_t TIM1_cb_flag;
#define LOOP_INTERVAL_TICKS 2000        // 2000 TIM1 ticks per second (500usec tick interval)

GPIO_PinState pushbutton_value, last_pushbutton_value;
int pushbutton_flag;

/* Private user code ---------------------------------------------------------*/
void Timer_config(void);
void Timer_tests(void);
void ADC_Read(int count);
uint32_t Wait_until_tick(uint32_t tick, uint32_t max_wait);

/* Private structures ---------------------------------------------------------*/
typedef struct {
    uint8_t enabled;
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    uint8_t pwm_setting;
} Pin_pwm_t;

Pin_pwm_t pwm_pb6;
Pin_pwm_t pwm_pb7;

float temperature_cal;
    
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
    uint32_t start_tick;
    uint32_t test_active = 0;
    
    HAL_Init();
  
    APP_SystemClockConfig(); 
	
	//Initilize needed periperhals
	GPIO_Init();
	TIMER_Init();
	UART_Init();
	ADC_Init();
    Timer_config();
    
  	start_tick = TIM1_cb_count;
    
    sprintf(outputStr, "NAATOS V2 PY32F003 Devkit tests.\r\n");		
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
    	
    while (1) 
    {
        //Timer_tests();		
        Wait_until_tick(start_tick + LOOP_INTERVAL_TICKS, LOOP_INTERVAL_TICKS*2);
        start_tick += LOOP_INTERVAL_TICKS;
		if (test_active) {
            ADC_Read(loop_count);
            loop_count++;
        }
        
        if (pushbutton_flag == 1) {
            pushbutton_flag = 0;   

            if (test_active) {
                test_active = 0;
                pwm_pb6.enabled = 0;
                pwm_pb7.enabled = 0;    
                sprintf(outputStr, "Test stopped.\r\n");		
            } else {
                pwm_pb6.enabled = 1;
                pwm_pb7.enabled = 1;        
                sprintf(outputStr, "Test started.\r\n");		
                test_active = 1;
                loop_count = 0;
            }                
            HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
        }
    }    
}

// Wait_until_tick uses the Timer1 callback variable TIM1_cb_count for precise timing delays
uint32_t Wait_until_tick(uint32_t tick, uint32_t max_wait)
{
    uint32_t start_tick, current_tick, wait_time;
	start_tick = TIM1_cb_count;
    if (start_tick >= tick) return 0;
    
    do {
        current_tick = TIM1_cb_count;
        wait_time = current_tick - start_tick;
    } while (current_tick < tick  && wait_time < max_wait);
    return wait_time;
}

void Timer_config(void)
{
    TIM1_cb_count = 0;
    TIM1_cb_flag = 0;
    
    HAL_SetTickFreq(1);     // set the tick timer to 1 MHz (1 msec per tick)    
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

	start_tick = TIM1_cb_count;
	tick_wait_count = Wait_until_tick(start_tick + 1, 10);
	end_tick = TIM1_cb_count;
	
    sprintf(outputStr, "Wait_until_tick start tick: %d end_tick: %d tick_wait_count: %d\r\n", start_tick, end_tick, tick_wait_count);		
	HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
}

#define VREFINT_V 1.20

float ADC_Vrefint_to_Vcc(uint32_t adc_val)
{
    float f;    
    f = (float) 4096.0 / (float) adc_val;
    return VREFINT_V * f;    
}

float ADC_to_Volts(uint32_t adc_val, float Vcc)
{
    float f;    
    f = (float) adc_val / (float) 4096.0;
    return Vcc * f;        
}

// PY32 temperature conversion formula:
// temp_c = ((85c - 30c) / (HAL_ADC_TSCAL2 - HAL_ADC_TSCAL1)) * (ADC_TEMP - HAL_ADC_TSCAL1) - 30
//      The first part of this equation is pre-calculated and stored in: temperature_cal
float ADC_Temp_to_degC(uint32_t adc_temp)
{
    float temp_c;    
    temp_c = temperature_cal * ((float) adc_temp - (float) HAL_ADC_TSCAL1) + 30.0f;
    return temp_c;
}

void ADC_Read(int count)
{
    float temp_c, vcc, pa2_v;
    
    //Sample with ADC in polling mode
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle, 1000);
    adcReading[0] = HAL_ADC_GetValue(&AdcHandle); // PA2
    
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle, 1000);
    adcReading[1] = HAL_ADC_GetValue(&AdcHandle); // MCU Temp sensor
    
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle, 1000);
    adcReading[2] = HAL_ADC_GetValue(&AdcHandle); // MCU VrefInt

    vcc = ADC_Vrefint_to_Vcc(adcReading[2]);
    temp_c = ADC_Temp_to_degC(adcReading[1]);
    pa2_v = ADC_to_Volts (adcReading[0], vcc);
    
    sprintf(outputStr, "%4d ADC: %u %u %u Vcc: %1.2f Temp: %1.2f PA2: %1.2f\r\n", count, adcReading[0], adcReading[1], adcReading[2], vcc, temp_c, pa2_v);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
}

void GPIO_Init(void)
{
	//Configure GPIO for ADC, UART, PWM outputs

    // Devkit pins:
	//  PA0 is TX
	//  PA1 is RX
    //  PA2 is ADC in
    //  PA12 is the pushbutton input
    //  PB5 is the LED
    //  PB6 is PWM output1
    //  PB7 is PWM output2
	
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
	HAL_GPIO_Init(GPIOA, &GpioInitStruct);
	
	//Pin settings for ADC input
	__HAL_RCC_ADC_CLK_ENABLE();
	
	AdcPinStruct.Pin = GPIO_PIN_2;
	AdcPinStruct.Mode = GPIO_MODE_ANALOG;
	AdcPinStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &AdcPinStruct);    
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);    

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);    
    
    pwm_pb6.enabled = 0;
    pwm_pb6.GPIOx = GPIOB;
    pwm_pb6.GPIO_Pin = 6;
    pwm_pb6.pwm_setting = 192;   
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    
    pwm_pb7.enabled = 0;
    pwm_pb7.GPIOx = GPIOB;
    pwm_pb7.GPIO_Pin = 7;
    pwm_pb7.pwm_setting = 192;   

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);    
    pushbutton_value = GPIO_PIN_SET;
    last_pushbutton_value = GPIO_PIN_SET;
    pushbutton_flag = 0;    
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
    // Control the PB6 PWM output
    // This is aligned with the start of the 125 usec PWM period
    if (pwm_pb6.enabled) {
        if ((TIM1_cb_count & 0xFF)  < pwm_pb6.pwm_setting)
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    }

    // Control the PB7 PWM output
    // This is aligned with the end of the 125 usec PWM period
    if (pwm_pb7.enabled) {
        if ((TIM1_cb_count & 0xFF)  > (255 - pwm_pb7.pwm_setting))
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    }
    
    // Latch the pushbutton_flag when the button is pressed (falling edge of PA12)
    pushbutton_value = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
    if (pushbutton_value == GPIO_PIN_RESET && last_pushbutton_value == GPIO_PIN_SET) pushbutton_flag = 1;
    last_pushbutton_value = pushbutton_value;    

    // blink the LED (at 12.5% duty cycle) if either PWM is enabled:
    if ((pwm_pb6.enabled > 0 || (pwm_pb7.enabled > 0)) && (TIM1_cb_count & 0x100) < 0x80) {
        //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
        if ((TIM1_cb_count & 0x7) == 0) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    }
    
    TIM1_cb_count++;
    TIM1_cb_flag = 1;
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
	AdcHandle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_HSI_DIV8; //HSI = 24 MHz (ADC Clock = 24 MHz / 8)
	AdcHandle.Init.Resolution = ADC_RESOLUTION_12B; //12 bits
	AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT; //Right aligned
	AdcHandle.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD; //Don't plan to use
	AdcHandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV; //single conversion
	AdcHandle.Init.LowPowerAutoWait = DISABLE; //use all the power
	AdcHandle.Init.ContinuousConvMode = DISABLE; //don't need for polling
	AdcHandle.Init.DiscontinuousConvMode = ENABLE; 
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

    //ADC_CHANNEL_2             (PA2)
    //ADC_CHANNEL_TEMPSENSOR    (ch 11, 9 usec min sample time)
    //ADC_CHANNEL_VREFINT       (ch 12)
	
	// Set ADC rank and channel
	AdcChanConf.Rank = ADC_CHANNEL_2; 
	AdcChanConf.Channel = ADC_CHANNEL_2;     
	
	if (HAL_ADC_ConfigChannel(&AdcHandle, &AdcChanConf) != HAL_OK)
	{
		APP_ErrorHandler();
	}
	// Set ADC rank and channel
	AdcChanConf.Rank = ADC_CHANNEL_TEMPSENSOR; 
	AdcChanConf.Channel = ADC_CHANNEL_TEMPSENSOR;     
	
	if (HAL_ADC_ConfigChannel(&AdcHandle, &AdcChanConf) != HAL_OK)
	{
		APP_ErrorHandler();
	}
    
    // precalculate part of the temperature calculation:
    temperature_cal = (float) (85 - 30) / (float) (HAL_ADC_TSCAL2 - HAL_ADC_TSCAL1);
    
	// Set ADC rank and channel
	AdcChanConf.Rank = ADC_CHANNEL_VREFINT; 
	AdcChanConf.Channel = ADC_CHANNEL_VREFINT;     
	
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
