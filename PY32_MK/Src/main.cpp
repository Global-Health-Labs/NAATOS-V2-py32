/*
*   File: main.c
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

#include "state_machine.h"
#include "alarm.h"
#include "pid.h"
#include "adc.h"
#include "app_data.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandle;
GPIO_InitTypeDef GpioInitStruct;
GPIO_InitTypeDef AdcPinStruct;
TIM_HandleTypeDef tim1Handle;

char outputStr[100];
app_data_t data;
flags_t flags;

uint32_t TIM1_cb_count;
uint32_t TIM1_cb_flag;

GPIO_PinState pushbutton_value, last_pushbutton_value;
int pushbutton_flag;

#define SH_FIXED_PWM_TEST 128
#define VH_FIXED_PWM_TEST 255

/*CONTROL structure
  holds process steps for each STATE in application. Each [INDEX] maps to enum state_machine
*/

#define PID_SH_P_TERM 60
#define PID_SH_I_TERM 0.5
#define PID_SH_D_TERM 0.333
#define PID_VH_P_TERM 60
#define PID_VH_I_TERM 0.5
#define PID_VH_D_TERM 0.333

CONTROL sample_amp_control[numProcess] = 
{
  {HEATER_SHUTDOWN_C, 0, 0, 2, 1, .5},
  {SAMPLE_ZONE_AMP_SOAK_TARGET_C, 0,0, PID_SH_P_TERM, PID_SH_I_TERM, PID_SH_D_TERM},
  {SAMPLE_ZONE_AMP_SOAK_TARGET_C, 0,0, PID_SH_P_TERM, PID_SH_I_TERM, PID_SH_D_TERM},
  {SAMPLE_ZONE_VALVE_SOAK_TARGET_C, 0, 0, PID_SH_P_TERM, PID_SH_I_TERM, PID_SH_D_TERM},
  {HEATER_SHUTDOWN_C, 0, 0, 2, 5, 1}
};
CONTROL valve_amp_control[numProcess] = 
{
  {HEATER_SHUTDOWN_C, 0, 0, 0, 0, 0},
  {VALVE_ZONE_AMP_SOAK_TARGET_C, 0,0, PID_VH_P_TERM, PID_VH_I_TERM, PID_VH_D_TERM},
  {VALVE_ZONE_VALVE_PREP_TARGET_C, 0,0, PID_VH_P_TERM, PID_VH_I_TERM, PID_VH_D_TERM},
  {VALVE_ZONE_VALVE_SOAK_TARGET_C,0, 0, PID_VH_P_TERM, PID_VH_I_TERM, PID_VH_D_TERM},
  {HEATER_SHUTDOWN_C, 0, 0, 0, 0, 0}
};



/* Private user code ---------------------------------------------------------*/
void Data_init(void);
void Timer_config(void);
uint32_t Wait_until_tick(uint32_t tick, uint32_t max_wait);
void Distribute_PWM_Bits(uint8_t pwm_val, uint64_t *pwm_bit_array);

typedef struct {
    uint8_t enabled;
    uint8_t pwm_setting;
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    uint16_t pwm_state;
    uint64_t pwm_bits[4];
} Pin_pwm_t;

Pin_pwm_t pwm_pb6;
Pin_pwm_t pwm_pb7;
    

static void APP_SystemClockConfig(void);
void TIMER_Init(void);

// PID structure holder
pid_controller_t sample_zone;
pid_controller_t valve_zone;

// Reinit Controller based on STATE MACHINE
void pid_init(pid_controller_t &pid, CONTROL pid_settings){
  pid_controller_init(
    &pid, 
    pid_settings.setpoint,
    pid_settings.kp,
    pid_settings.ki,
    pid_settings.kd,
    ATTINY_8BIT_PWM_MAX,
    SLEW_RATE_LIMIT);
}

void system_setup() {
    HAL_Init();
  
    APP_SystemClockConfig(); 
	
	//Initilize needed periperhals
	GPIO_Init();
	TIMER_Init();
	UART_Init();
	ADC_Init();
    Data_init();
    Timer_config();

    sprintf(outputStr, "NAATOS V2 PY32F003 MK. %s %s\r\n", FW_VERSION_STR, BUILD_HW_STR);		
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
    
    // INIT PID Structure
    pid_init(sample_zone,sample_amp_control[data.state]);
    pid_init(valve_zone,valve_amp_control[data.state]);
}

/**
  * @brief  
  * @retval int
  */
int main(void)
{
    uint32_t loop_count = 0;
    uint32_t start_tick;
    uint32_t test_active = 0;
    
    system_setup();
    
  	start_tick = TIM1_cb_count;    
    	
    //Distribute_PWM_Bits((uint8_t) 3, (uint64_t *) pwm_pb6.pwm_bits);    
    //Distribute_PWM_Bits((uint8_t) 7, (uint64_t *) pwm_pb6.pwm_bits);    
    //Distribute_PWM_Bits((uint8_t) 254, (uint64_t *) pwm_pb6.pwm_bits);    
    
    while (1) 
    {
        Wait_until_tick(start_tick + TICKS_PER_SEC, TICKS_PER_SEC*2);
        start_tick += TICKS_PER_SEC;
		if (test_active) {
            if (pwm_pb6.enabled && pwm_pb6.pwm_setting > 0) {
                while (pwm_pb6.pwm_state < 10);        // wait until the next active PWM cycle
            }
            ADC_Read();
            print_log_data();
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

void Data_init(void)
{    
    flags.flagDataCollection = false;
    flags.flagUpdateTemperature = false;
    flags.flagUpdatePID = false;
    flags.flagSendLog = false;
    flags.flagUpdateLed = false;
    flags.flagDelayedStart = false;
    
    data.state = low_power;
    data.alarm = no_alarm;
    
    data.sample_heater_pwm_value = 0;
    data.valve_heater_pwm_value = 0;
    data.sample_temperature_c = 0;
    data.valve_temperature_c = 0;
    data.py32_temperature_c = 0;
    data.test_adc_voltage = 0;
    data.current_time_msec = 0;
    data.battery_voltage = 0;
    data.valve_max_temperature_c = 0;
    data.valve_ramp_time = 0;    
}

void print_log_data(void) 
{
    sprintf(outputStr, "%4d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %d\r\n", data.current_time_msec, data.sample_temperature_c, sample_zone.setpoint, data.sample_heater_pwm_value, data.valve_temperature_c, valve_zone.setpoint, data.valve_heater_pwm_value, data.battery_voltage, data.state);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
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


#define PWM_BITS 8
#define PWM_BITFIELD_SIZE 256

/**
 * Shifts a 256-bit unsigned integer (represented as an array of four 64-bit unsigned integers)
 * to the left by shift_val bits.
 *
 * Parameters:
 * - input (uint64_t*): Pointer to an array of four 64-bit unsigned integers.
 * - shift_val (uint8_t): Number of bits to shift to the left (0-255).
 *
 * Returns:
 * - The input array is modified in place to reflect the shifted value.
 */
void Shift_left_256(uint64_t *input, uint8_t shift_val) {

    if (shift_val == 0) {
        return;
    }

    int word_shift = shift_val / 64;      // Number of 64-bit words to shift
    int bit_shift = shift_val % 64;       // Number of bits to shift within words

    // Temporary array to store the result
    uint64_t result[4] = {0, 0, 0, 0};

    // Perform the bit shift
    for (int i = 3; i >= 0; i--) {
        if (i - word_shift >= 0) {
            result[i] = input[i - word_shift] << bit_shift;
            if (bit_shift > 0 && i - word_shift - 1 >= 0) {
                // Add the carry bits from the lower word
                result[i] |= input[i - word_shift - 1] >> (64 - bit_shift);
            }
        }
    }

    // Copy (OR) the result back to the input array
    for (int i = 0; i < 4; i++) {
        input[i] |= result[i];
    }
}

// Distribute_PWM_Bits takes an 8 bit PWM value and distributes the bits (energy) evenly across 256 timeslots.
void Distribute_PWM_Bits(uint8_t pwm_val, uint64_t *pwm_bit_array)
{
    uint64_t sub_bitfield[4];
    uint32_t data_pos, bit_pos;   
    uint32_t base_val, shift;

    sprintf(outputStr, "Distribute_PWM_Bits %d\r\n", pwm_val);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);        
    
    pwm_bit_array[0] = 0;
    pwm_bit_array[1] = 0;
    pwm_bit_array[2] = 0;
    pwm_bit_array[3] = 0;
    
    if (pwm_val == 0x0) return;
    else if (pwm_val == 0xFF) {
        pwm_bit_array[0] = 0xFFFFFFFFFFFFFFFF;
        pwm_bit_array[1] = 0xFFFFFFFFFFFFFFFF;
        pwm_bit_array[2] = 0xFFFFFFFFFFFFFFFF;
        pwm_bit_array[3] = 0xFFFFFFFFFFFFFFFF;
        return;
    }    
    
    data_pos = 1;    
    for (uint32_t i=0; i< PWM_BITS; i++) {
        bit_pos = PWM_BITS - i - 1;
        if (((pwm_val>>bit_pos) & 0x1) == 0x01) {
            base_val = PWM_BITFIELD_SIZE / (1<<bit_pos);
            sub_bitfield[0] = 0;
            sub_bitfield[1] = 0;
            sub_bitfield[2] = 0;
            sub_bitfield[3] = 0;
            if (data_pos < 64) sub_bitfield[0] = (uint64_t)(1<<data_pos);
            else if (data_pos < 2*64) sub_bitfield[1] = (uint64_t)(1<<(data_pos-64));
            else if (data_pos < 3*64) sub_bitfield[2] = (uint64_t)(1<<(data_pos-2*64));
            else sub_bitfield[3] = (uint64_t)(1<<(data_pos-3*64));

            shift = base_val;
            for (uint32_t j=1; j < (bit_pos+1); j++) {
                //sub_bitfield[0] |= (sub_bitfield[0] << shift);
                Shift_left_256(sub_bitfield, shift);
                shift *= 2;
            }           
            pwm_bit_array[0] |= sub_bitfield[0];
            pwm_bit_array[1] |= sub_bitfield[1];
            pwm_bit_array[2] |= sub_bitfield[2];
            pwm_bit_array[3] |= sub_bitfield[3];
        }
        data_pos *= 2;
    }    
    
    sprintf(outputStr, "0: %32llx\r\n", pwm_bit_array[0]);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);        
    sprintf(outputStr, "1: %32llx\r\n", pwm_bit_array[1]);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);        
    sprintf(outputStr, "2: %32llx\r\n", pwm_bit_array[2]);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);        
    sprintf(outputStr, "3: %32llx\r\n", pwm_bit_array[3]);
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
	
	AdcPinStruct.Pin = GPIO_PIN_1;
	AdcPinStruct.Mode = GPIO_MODE_ANALOG;
	AdcPinStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &AdcPinStruct);    

	AdcPinStruct.Pin = GPIO_PIN_2;
	AdcPinStruct.Mode = GPIO_MODE_ANALOG;
	AdcPinStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &AdcPinStruct);    

	AdcPinStruct.Pin = GPIO_PIN_3;
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
    pwm_pb6.pwm_state = 0;
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
    pwm_pb7.pwm_state = 0;

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
        if ((TIM1_cb_count & 0xFF)  < pwm_pb6.pwm_setting) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
            pwm_pb6.pwm_state++;
        } else {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
            pwm_pb6.pwm_state = 0;
        }
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        pwm_pb6.pwm_state = 0;
    }

    // Control the PB7 PWM output
    // This is aligned with the end of the 125 usec PWM period
    if (pwm_pb7.enabled) {
        if ((TIM1_cb_count & 0xFF)  > (255 - pwm_pb7.pwm_setting)) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
            pwm_pb7.pwm_state++;
        } else {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
            pwm_pb7.pwm_state = 0;
        }
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
        pwm_pb7.pwm_state = 0;
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

void APP_ErrorHandler(void)
{
  while (1)
  {
  }
}



/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
