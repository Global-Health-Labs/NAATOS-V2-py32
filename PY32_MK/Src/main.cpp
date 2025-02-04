/*
*   File: main.cpp
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
#include "timers.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandle;
GPIO_InitTypeDef GpioInitStruct;
GPIO_InitTypeDef AdcPinStruct;

char outputStr[100];
app_data_t data;
flags_t flags;

int8_t PWMTimerNumber;
int8_t LEDTimerNumber;
int8_t SecondTimerNumber;
int8_t MinuteTimerNumber;
int8_t DelayStartTimerNumber;
int8_t DataCollectionTimerNumber;
int8_t DelayedStartTimerNumber;
int8_t PushbuttonTimerNumber;

GPIO_PinState pushbutton_value, last_pushbutton_value;

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
void Distribute_PWM_Bits(uint8_t pwm_val, uint64_t *pwm_bit_array);

void start_naat_test(void);
void stop_naat_test(void);
void naat_test_control(void);

Pin_pwm_t pwm_pb6;
Pin_pwm_t pwm_pb7; 

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
    PWM_MAX,
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

    sprintf(outputStr, "NAATOS V2 PY32F003 MK. %s %s\r\n", FW_VERSION_STR, BUILD_HW_STR);		
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
    
    // set up the timers:
    PWMTimerNumber = Register_timer(PWMTimer_ISR,  PWM_TIMER_INTERVAL);
    LEDTimerNumber = Register_timer(LEDTimer_ISR,  LED_TIMER_INTERVAL);    
    SecondTimerNumber = Register_timer(SecondTimer_ISR,  SECOND_TIMER_INTERVAL);
    MinuteTimerNumber = Register_timer(MinuteTimer_ISR,  MINUTE_TIMER_INTERVAL);
    DataCollectionTimerNumber = Register_timer(DataCollection_ISR,  DATA_COLLECTION_TIMER_INTERVAL);
    DelayedStartTimerNumber = Register_timer(DelayedStart_ISR,  STARTUP_DELAY_MS);
    PushbuttonTimerNumber = Register_timer(Pushbutton_ISR,  PUSHBUTTON_TIMER_INTERVAL);
    
    // INIT PID Structure
    pid_init(sample_zone,sample_amp_control[data.state]);
    pid_init(valve_zone,valve_amp_control[data.state]);
    
    // start the timers:
    Enable_timer(LEDTimerNumber);
    Enable_timer(SecondTimerNumber);
    Enable_timer(MinuteTimerNumber);
    Enable_timer(PushbuttonTimerNumber);
    Enable_timer(DataCollectionTimerNumber);
    Enable_timer(PWMTimerNumber);
}

/**
  * @brief  
  * @retval int
  */
int main(void)
{
    //uint32_t start_tick;
    
    system_setup();
    
  	//start_tick = TIM1_tick_count;    
    	
    //Distribute_PWM_Bits((uint8_t) 3, (uint64_t *) pwm_pb6.pwm_bits);    
    //Distribute_PWM_Bits((uint8_t) 7, (uint64_t *) pwm_pb6.pwm_bits);    
    //Distribute_PWM_Bits((uint8_t) 254, (uint64_t *) pwm_pb6.pwm_bits);    
    
    while (1) 
    {
        //Wait_until_tick(start_tick + TICKS_PER_SEC, TICKS_PER_SEC*2);
        //start_tick += TICKS_PER_SEC;
        if (flags.flag_1second) {
            flags.flag_1second = false;
            
            if (data.test_active) {
                naat_test_control();
            }
        }
        
        if (flags.flagPushbutton) {
            flags.flagPushbutton = false;   

            if (data.test_active) {
                stop_naat_test();
            } else {
                start_naat_test();
            }                
        }
    }    
}

void start_naat_test(void) {
    data.msec_test_count = 0;
    data.state = low_power;
    
    data.sample_heater_pwm_value = 192;
    data.valve_heater_pwm_value = 192;
    pwm_pb6.pwm_setting = data.sample_heater_pwm_value;   
    pwm_pb7.pwm_setting = data.valve_heater_pwm_value;   
    
    pwm_pb6.enabled = 1;
    pwm_pb7.enabled = 1;        
    data.test_active = true;
    sprintf(outputStr, "Test started.\r\n");		   
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
}

void stop_naat_test(void) {
    data.test_active = false;
    data.state = low_power;
    data.sample_heater_pwm_value = 0;
    data.valve_heater_pwm_value = 0;
    pwm_pb6.pwm_setting = data.sample_heater_pwm_value;   
    pwm_pb7.pwm_setting = data.valve_heater_pwm_value;   
    pwm_pb6.enabled = 0;
    pwm_pb7.enabled = 0;    
    sprintf(outputStr, "Test stopped.\r\n");		
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
}

void naat_test_control(void) {
    if (pwm_pb6.enabled && pwm_pb6.pwm_setting > 0) {
        while (pwm_pb6.pwm_state < 10);        // wait until the next active PWM cycle
    }
    ADC_Read();
    print_log_data();
}

void Data_init(void)
{    
    flags.flagDataCollection = false;
    flags.flagUpdatePID = false;
    flags.flagUpdateLED = false;
    flags.flagDelayedStart = false;
    flags.flagPushbutton = false;
    flags.flag_1second = false;
    flags.flag_1minute = false;
    
    data.test_active = false;
    data.state = low_power;
    data.alarm = no_alarm;
    
    data.sample_heater_pwm_value = 0;
    data.valve_heater_pwm_value = 0;
    data.sample_temperature_c = 0;
    data.valve_temperature_c = 0;
    data.py32_temperature_c = 0;
    data.test_adc_voltage = 0;
    data.msec_tick_count = 0;
    data.msec_test_count = 0;
    data.battery_voltage = 0;
    data.valve_max_temperature_c = 0;
    data.valve_ramp_time = 0;    
}

void print_log_data(void) 
{
    sprintf(outputStr, "%4d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %d\r\n", data.msec_test_count, data.sample_temperature_c, sample_zone.setpoint, data.sample_heater_pwm_value, data.valve_temperature_c, valve_zone.setpoint, data.valve_heater_pwm_value, data.battery_voltage, data.state);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
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
    pwm_pb6.pwm_setting = 0;   
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
    pwm_pb7.pwm_setting = 0;   
    pwm_pb7.pwm_state = 0;

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);    
    pushbutton_value = GPIO_PIN_SET;
    last_pushbutton_value = GPIO_PIN_SET;
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

// Timer ISR functions are called from the HAL_TIM ISR.
// Only time critical code should be run in these routines.
// It is recommended to set a flag in the ISR that enables lower priority code to run in user space.

void PWMTimer_ISR(void)
{   
    // Control the PB6 PWM output (sample heater)
    // This is aligned with the start of the 125 usec PWM period
    if (pwm_pb6.enabled) {
        if ((TIM1_tick_count & 0xFF)  < pwm_pb6.pwm_setting) {
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

    // Control the PB7 PWM output (valve heater)
    // This is aligned with the end of the 125 usec PWM period
    if (pwm_pb7.enabled) {
        if ((TIM1_tick_count & 0xFF)  > (255 - pwm_pb7.pwm_setting)) {
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
        
}

void LEDTimer_ISR(void)
{
    if (data.test_active) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    }
    
    /*
    // blink the LED (at 12.5% duty cycle) if either PWM is enabled:
    if ((pwm_pb6.enabled > 0 || (pwm_pb7.enabled > 0)) && (TIM1_tick_count & 0x100) < 0x80) {
        //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
        if ((TIM1_tick_count & 0x7) == 0) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    }
    */
    
}
void SecondTimer_ISR(void)
{
    flags.flag_1second = true;
}

void MinuteTimer_ISR(void)
{
    flags.flag_1minute = true;
}

void DataCollection_ISR(void)
{
    flags.flagDataCollection = true;
}

void DelayedStart_ISR(void)
{
    flags.flagDelayedStart = true;
}

void Pushbutton_ISR(void)
{
    // Latch the pushbutton_flag when the button is pressed (falling edge of PA12)
    pushbutton_value = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
    if (pushbutton_value == GPIO_PIN_RESET && last_pushbutton_value == GPIO_PIN_SET) flags.flagPushbutton = true;
    last_pushbutton_value = pushbutton_value;    
}

void APP_ErrorHandler(void)
{
  while (1)
  {
  }
}



/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
