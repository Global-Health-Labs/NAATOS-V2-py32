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

Pin_assignments_t Pins;

char outputStr[200];
app_data_t data;
flags_t flags;

int8_t PWMTimerNumber;
int8_t LEDTimerNumber;
int8_t PIDTimerNumber;
int8_t MinuteTimerNumber;
int8_t DelayStartTimerNumber;
int8_t DataCollectionTimerNumber;
int8_t DelayedStartTimerNumber;
int8_t PushbuttonTimerNumber;
int8_t LogTimerNumber;

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

// Heater control structures
Pin_pwm_t pwm_amp_ctrl;
Pin_pwm_t pwm_valve_ctrl; 

// PID structure holder
pid_controller_t sample_zone;
pid_controller_t valve_zone;

// Init PID Controller based on the STATE MACHINE state
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
	
	//Initialize periperhals
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
    PIDTimerNumber = Register_timer(PIDTimer_ISR,  PID_TIMER_INTERVAL);
    MinuteTimerNumber = Register_timer(MinuteTimer_ISR,  MINUTE_TIMER_INTERVAL);
    DataCollectionTimerNumber = Register_timer(DataCollection_ISR,  DATA_COLLECTION_TIMER_INTERVAL);
    DelayedStartTimerNumber = Register_timer(DelayedStart_ISR,  STARTUP_DELAY_MS);
    PushbuttonTimerNumber = Register_timer(Pushbutton_ISR,  PUSHBUTTON_TIMER_INTERVAL);
    LogTimerNumber = Register_timer(LogData_ISR,  LOG_TIMER_INTERVAL);
    
    // INIT PID Structure
    pid_init(sample_zone,sample_amp_control[data.state]);
    pid_init(valve_zone,valve_amp_control[data.state]);
    
    // start the timers:
    Enable_timer(LEDTimerNumber);
    Enable_timer(MinuteTimerNumber);
    Enable_timer(PushbuttonTimerNumber);
    Enable_timer(DataCollectionTimerNumber);
    
    HAL_GPIO_WritePin(Pins.GPIOx_LED2, Pins.GPIO_Pin_LED2, GPIO_PIN_RESET); // Turn on LED2
    
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

#ifdef DEBUG    
    //Distribute_PWM_Bits((uint8_t) 3, (uint64_t *) pwm_amp_ctrl.pwm_bits);    
    //Distribute_PWM_Bits((uint8_t) 7, (uint64_t *) pwm_amp_ctrl.pwm_bits);    
    //Distribute_PWM_Bits((uint8_t) 254, (uint64_t *) pwm_amp_ctrl.pwm_bits);   
#endif

    sprintf(outputStr, "HAL_ADC_TSCAL1 %d HAL_ADC_TSCAL2 %d\r\n", HAL_ADC_TSCAL1, HAL_ADC_TSCAL2);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
    
    ADC_Read();
    sprintf(outputStr, "ADC: %d %d %d %d %d %d %d\r\n", data.adcReading[0], data.adcReading[1], data.adcReading[2], data.adcReading[3], data.adcReading[4], data.adcReading[5], data.adcReading[6]);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
    sprintf(outputStr, "system_input_voltage: %1.2f vcc_mcu_voltage: %1.2f py32_temperature_c: %1.2f\r\n", data.system_input_voltage, data.vcc_mcu_voltage, data.py32_temperature_c);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
    sprintf(outputStr, "usb_cc1_voltage: %1.2f usb_cc2_voltage: %1.2f\r\n", data.usb_cc1_voltage, data.usb_cc2_voltage);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
    sprintf(outputStr, "sample_temperature_v: %1.2f sample_thermistor_r: %d sample_temperature_c: %1.2f\r\n", data.sample_thermistor_v, data.sample_thermistor_r, data.sample_temperature_c);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
    sprintf(outputStr, "valve_temperature_v: %1.2f valve_thermistor_r: %d valve_temperature_c: %1.2f\r\n\r\n", data.valve_thermistor_v, data.valve_thermistor_r, data.valve_temperature_c);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    

    ADC_Set_USB_cc_read_state(false);       // disable reading of USB-C CC voltages in the ADC.
    
    while (1) 
    {
        if (flags.flagPushbutton) {
            flags.flagPushbutton = false;   

            if (data.test_active) {
                stop_naat_test();
            } else {
                data.msec_test_count = 0;
                data.minute_test_count = 0;
                Enable_timer(DelayedStartTimerNumber);
                Enable_timer(LogTimerNumber);                
            }                
        }
        
        if (flags.flagDelayedStart) {
            flags.flagDelayedStart = false;
            Disable_timer(DelayedStartTimerNumber);
            
            // Start the test:
            start_naat_test();
            Enable_timer(PWMTimerNumber);
        }                

        if (flags.flag_1minute) {
            flags.flag_1minute = false;
            data.minute_test_count++;
            
            if (data.minute_test_count >= AMPLIFICATION_TIME_MIN) {
                if (data.minute_test_count >= AMPLIFICATION_TIME_MIN + ACUTATION_TIME_MIN) {
                    // in detection
                    if (data.state != detection) {
                        data.state = detection;
                        pid_init(sample_zone,sample_amp_control[data.state]);
                        pid_init(valve_zone,valve_amp_control[data.state]);
                        // turn both LEDs off during detection
                    }
                    if (data.minute_test_count  >= AMPLIFICATION_TIME_MIN + ACUTATION_TIME_MIN + DETECTION_TIME_MIN) {
                        // final state -- END
                        if (data.state != low_power) {
                            // SHUT DOWN LOADS!!!!
                            data.state = low_power;
                            pid_init(sample_zone,sample_amp_control[data.state]);
                            pid_init(valve_zone,valve_amp_control[data.state]);
                            stop_naat_test();
        
                            send_vh_max_temp();
                            if (data.valve_max_temperature_c < VALVE_ZONE_MIN_VALID_TEMP_C) {
                                // blink both LEDs to indicate that the minimum valve temperature was not reached.
                                data.alarm = valve_min_temp_not_reached;
                            } 
                            // change LEDs to opposite states so they can blink opposite of each other (in alarm)
        
                            // Enable the LED timer so they can blink at the end
                        }
                    }
                } else if (data.minute_test_count >= AMPLIFICATION_TIME_MIN) {
                    if (data.state != actuation) {
                        data.state = actuation;
                        pid_init(sample_zone,sample_amp_control[data.state]);
                        pid_init(valve_zone,valve_amp_control[data.state]);
        
                        // Change UI (both LEDs are on during valve activation)
                    }
                }
            } else if (data.minute_test_count >= AMPLIFICATION_TIME_MIN - ACUTATION_PREP_TIME_MIN) {
                  // Pre-heat VH during the last minute of amplification.
                  if (data.state != actuation_prep) {
                      data.state = actuation_prep;
                      pid_init(sample_zone,sample_amp_control[data.state]);
                      pid_init(valve_zone,valve_amp_control[data.state]);
                  }        
            } 
        }

        /*UPDATE INPUT::TEMPERATURE SENSORS*/
        if (flags.flagDataCollection){
          flags.flagDataCollection = false;

          // Turn off (suspend) the heaters while reading the ADCs (to minimize noise)
          if (pwm_amp_ctrl.enabled && pwm_amp_ctrl.pwm_setting > 0) {              
              pwm_amp_ctrl.enabled = false;
              pwm_amp_ctrl.suspended = true;
              HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL1, Pins.GPIO_Pin_AMP_CTRL1, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL2, Pins.GPIO_Pin_AMP_CTRL2, GPIO_PIN_RESET);
          }

          if (pwm_valve_ctrl.enabled && pwm_amp_ctrl.pwm_setting > 0) {              
              pwm_valve_ctrl.enabled = false;      
              pwm_valve_ctrl.suspended = true;
              HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL1, Pins.GPIO_Pin_VALVE_CTRL1, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL2, Pins.GPIO_Pin_VALVE_CTRL2, GPIO_PIN_RESET);
          }
          
          ADC_Read();
          
          if (pwm_amp_ctrl.suspended) {
              pwm_amp_ctrl.suspended = false;
              pwm_amp_ctrl.enabled = true;
          }
          if (pwm_valve_ctrl.suspended) {
              pwm_valve_ctrl.suspended = false;
              pwm_valve_ctrl.enabled = true;
          }
          
          // Measure the number of seconds it takes to ramp to the minimum valve temperature:
          if (data.state == actuation && data.valve_ramp_time == 0 && data.valve_temperature_c >= VALVE_ZONE_MIN_VALID_TEMP_C) {
            data.valve_ramp_time = data.msec_test_count / 1000 - (AMPLIFICATION_TIME_MIN * 60);
          }
        }

        /*UPDATE OUTPUT:HEATER LOAD*/
        if (flags.flagUpdatePID) {
            flags.flagUpdatePID = false;
            if (data.test_active) {
                data.msec_test_count += (PID_TIMER_INTERVAL / TICKS_PER_MSEC);
            }
            
            //compute(&sample_zone,data.sample_temperature_c);
            pid_controller_compute(&sample_zone,data.sample_temperature_c);
            pid_controller_compute(&valve_zone,data.valve_temperature_c);
          
            //data.sample_heater_pwm_value = SH_FIXED_PWM_TEST;
            //data.valve_heater_pwm_value = VH_FIXED_PWM_TEST;
            data.sample_heater_pwm_value = sample_zone.out;
            data.valve_heater_pwm_value = valve_zone.out;
        }
        
        if (flags.flagSendLogData) {
            flags.flagSendLogData = false;
            print_log_data();
        }
    }    
}

void start_naat_test(void) {
    data.msec_test_count = 0;
    data.minute_test_count = 0;
    data.state = low_power;
    data.valve_max_temperature_c = 0;
    data.valve_ramp_time = 0;    
    
    data.sample_heater_pwm_value = 0;
    data.valve_heater_pwm_value = 0;
    pwm_amp_ctrl.heater_level_high = false;
    pwm_amp_ctrl.pwm_setting = data.sample_heater_pwm_value;       
    pwm_amp_ctrl.enabled = false;
    
    pwm_valve_ctrl.enabled = false;
    pwm_valve_ctrl.heater_level_high = false;

    HAL_GPIO_WritePin(Pins.GPIOx_LED2, Pins.GPIO_Pin_LED2, GPIO_PIN_SET); // Turn off LED2    
    
    data.test_active = true;
    if (data.state != amplification) {
        data.state = amplification;
        pid_init(sample_zone,sample_amp_control[data.state]);
        pid_init(valve_zone,valve_amp_control[data.state]);
        sprintf(outputStr, "Starting Test\r\n");		   
    } else {
        sprintf(outputStr, "Err: Invalid test starting state.\r\n");		   
    }
      
    Enable_timer(PIDTimerNumber);
    
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
}

void stop_naat_test(void) {
    data.test_active = false;
    data.state = low_power;
    data.sample_heater_pwm_value = 0;
    data.valve_heater_pwm_value = 0;
    
    pwm_amp_ctrl.pwm_setting = 0;   
    pwm_amp_ctrl.enabled = false;
    
    pwm_valve_ctrl.pwm_setting = 0;   
    pwm_valve_ctrl.enabled = false;
    
    Disable_timer(LogTimerNumber);                
    Disable_timer(MinuteTimerNumber);
    
    sprintf(outputStr, "Test stopped.\r\n");		
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
}

void Data_init(void)
{    
    flags.flagDataCollection = false;
    flags.flagUpdatePID = false;
    flags.flagUpdateLED = false;
    flags.flagDelayedStart = false;
    flags.flagPushbutton = false;
    flags.flag_1minute = false;
    
    data.test_active = false;
    data.state = low_power;
    data.alarm = no_alarm;
    data.minute_test_count = 0;
    
    data.sample_heater_pwm_value = 0;
    data.valve_heater_pwm_value = 0;
    data.sample_temperature_c = 0;
    data.valve_temperature_c = 0;
    data.py32_temperature_c = 0;
    data.msec_tick_count = 0;
    data.msec_test_count = 0;
    data.vcc_mcu_voltage = 0;
    data.system_input_voltage = 0;
    data.valve_max_temperature_c = 0;
    data.valve_ramp_time = 0;    
}

void print_log_data(void) 
{
    sprintf(outputStr, "%4d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %d\r\n", data.msec_test_count, data.sample_temperature_c, sample_zone.setpoint, data.sample_heater_pwm_value, data.valve_temperature_c, valve_zone.setpoint, data.valve_heater_pwm_value, data.system_input_voltage, data.state);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
}

void send_vh_max_temp(void) {
    sprintf(outputStr, "VH_MAX: %1.2f valve_ramp_time: %d\r\n", data.valve_max_temperature_c, data.valve_ramp_time);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
}


#define PWM_BITS 8
#define PWM_BITFIELD_SIZE 256

#ifdef DEBUG    

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
    
#endif 

void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

	//Configure GPIO for ADC, UART, PWM outputs

    // Devkit (benchtop prototype) pins:
	//  PIN 19, PA0 is TX
	//  PIN 20, PA1 is RX
    //  PIN 1, PA2 is ADC in: AMP_TEMP_V
    //  PIN 2, PA3 is ADC in: VALVE_TEMP_V
    //  PIN 3, PA4 is ADC_SPARE 
    //  PIN 5, PA6 is ADC in: V_BATT_SENSE
    //  PIN 8, PA12 is the pushbutton input
    //  PIN 12, PB5 is the LED
    //  PIN 5, PB6 is PWM output1
    //  PIN 6, PB7 is PWM output2
    //  PIN 15, PF4 is BOOT0 jumper
    //  PIN 16, PF0 is OSCIN
    //  PIN 17, PF1 is OSCOUT
    
    
    // MK5 and MK6 pins:
	//  PIN 19, PA0 is TX
	//  PIN 20, PA1 is RX
    //  PIN 1, PA2 is ADC in: VALVE_TEMP_V
    //  PIN 2, PA3 is ADC in: AMP_TEMP_V
    //  PIN 8, PA4 is ADC in: USB_CC2
    //  PIN 4, PA5 is LED1
    //  PIN 12, PB5 is LED2
    //  PIN 5, PA6 is ADC in: V_BATT_SENSE
    //  PIN 6, PA7 is ADC in: USB_CC1
    //  PIN 13, PB6 is PWM output1: AMP_CTRL1
    //  PIN 14, PB7 is PWM output2: AMP_CTRL2
    //  PIN 16, PF0 is PWM output3: VALVE_CTRL1
    //  PIN 17, PF1 is PWM output4: VALVE_CTRL2
    //  PIN 18, PF2 is NRESET pushbutton (not used, pulled up)
    //  PIN 15, PF4 is BOOT0 (not used, pulled down)
    
	
	// Enable peripheral clocks
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
	
	// Initialize UART pins
	GpioInitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GpioInitStruct.Mode = GPIO_MODE_AF_PP;
	GpioInitStruct.Pull = GPIO_PULLUP;
	GpioInitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GpioInitStruct.Alternate = GPIO_AF9_USART2;
	HAL_GPIO_Init(GPIOA, &GpioInitStruct);
	
	//Pin settings for ADC inputs
	__HAL_RCC_ADC_CLK_ENABLE();
    
    // Device pin assignments:
#ifdef BOARDCONFIG_MK5_MK6 
    Pins.GPIOx_AMP_TEMP_V = GPIOA;
    Pins.GPIO_Pin_AMP_TEMP_V = GPIO_PIN_3;
    Pins.ADC_CHANNEL_AMP_TEMP_V = ADC_CHANNEL_3;    
    Pins.GPIOx_AMP_VALVE_TEMP_V = GPIOA;
    Pins.GPIO_Pin_AMP_VALVE_TEMP_V = GPIO_PIN_2;
    Pins.ADC_CHANNEL_VALVE_TEMP_V = ADC_CHANNEL_2;    
    Pins.GPIOx_AMP_V_BATT_SENSE = GPIOA;
    Pins.GPIO_Pin_V_BATT_SENSE = GPIO_PIN_6;
    Pins.ADC_CHANNEL_V_BATT_SENSE = ADC_CHANNEL_6;    
    Pins.GPIOx_USB_CC1 = GPIOA;
    Pins.GPIO_Pin_USB_CC1 = GPIO_PIN_7;
    Pins.ADC_CHANNEL_USB_CC1 = ADC_CHANNEL_7;    
    Pins.GPIOx_USB_CC2 = GPIOA;
    Pins.GPIO_Pin_USB_CC2 = GPIO_PIN_4;
    Pins.ADC_CHANNEL_USB_CC2 = ADC_CHANNEL_4;    
    Pins.GPIOx_LED1 = GPIOA;
    Pins.GPIO_Pin_LED1 = GPIO_PIN_5;
    Pins.GPIOx_LED2 = GPIOB;
    Pins.GPIO_Pin_LED2 = GPIO_PIN_5;
    Pins.GPIOx_AMP_CTRL1 = GPIOB;
    Pins.GPIO_Pin_AMP_CTRL1 = GPIO_PIN_6;
    Pins.GPIOx_AMP_CTRL2 = GPIOB;
    Pins.GPIO_Pin_AMP_CTRL2 = GPIO_PIN_7;
    Pins.GPIOx_VALVE_CTRL1 = GPIOF;
    Pins.GPIO_Pin_VALVE_CTRL1 = GPIO_PIN_0;
    Pins.GPIOx_VALVE_CTRL2 = GPIOF;
    Pins.GPIO_Pin_VALVE_CTRL2 = GPIO_PIN_1;
    Pins.GPIOx_PUSHBUTTON = GPIOF;
    Pins.GPIO_Pin_PUSHBUTTON = GPIO_PIN_2;
#else    
    Pins.GPIOx_AMP_TEMP_V = GPIOA;
    Pins.GPIO_Pin_AMP_TEMP_V = GPIO_PIN_2;
    Pins.ADC_CHANNEL_AMP_TEMP_V = ADC_CHANNEL_2;    
    Pins.GPIOx_AMP_VALVE_TEMP_V = GPIOA;
    Pins.GPIO_Pin_AMP_VALVE_TEMP_V = GPIO_PIN_3;
    Pins.ADC_CHANNEL_VALVE_TEMP_V = ADC_CHANNEL_3;    
    Pins.GPIOx_AMP_V_BATT_SENSE = GPIOA;
    Pins.GPIO_Pin_V_BATT_SENSE = GPIO_PIN_6;
    Pins.ADC_CHANNEL_V_BATT_SENSE = ADC_CHANNEL_6;    
    //Pins.GPIOx_USB_CC1 = GPIOA;
    //Pins.GPIO_Pin_USB_CC1 = GPIO_PIN_7;
    //Pins.ADC_CHANNEL_USB_CC1 = ADC_CHANNEL_7;    
    //Pins.GPIOx_USB_CC2 = GPIOA;
    //Pins.GPIO_Pin_USB_CC2 = GPIO_PIN_12;
    //Pins.ADC_CHANNEL_USB_CC2 = ADC_CHANNEL_12;    
    Pins.GPIOx_LED1 = GPIOB;
    Pins.GPIO_Pin_LED1 = GPIO_PIN_5;
    //Pins.GPIOx_LED2 = GPIOB;
    //Pins.GPIO_Pin_LED2 = GPIO_PIN_5;
    Pins.GPIOx_AMP_CTRL1 = GPIOB;
    Pins.GPIO_Pin_AMP_CTRL1 = GPIO_PIN_6;
    Pins.GPIOx_AMP_CTRL2 = GPIOB;
    Pins.GPIO_Pin_AMP_CTRL2 = GPIO_PIN_7;
    //Pins.GPIOx_VALVE_CTRL1 = GPIOF;
    //Pins.GPIO_Pin_VALVE_CTRL1 = GPIO_PIN_0;
    //Pins.GPIOx_VALVE_CTRL2 = GPIOF;
    //Pins.GPIO_Pin_VALVE_CTRL2 = GPIO_PIN_1;
    Pins.GPIOx_PUSHBUTTON = GPIOA;
    Pins.GPIO_Pin_PUSHBUTTON = GPIO_PIN_12;
#endif
	
	//AdcPinStruct.Pin = GPIO_PIN_1;              // PA1 / UART_RX alternative function
	//AdcPinStruct.Mode = GPIO_MODE_ANALOG;
	//AdcPinStruct.Pull = GPIO_NOPULL;
	//HAL_GPIO_Init(GPIOA, &AdcPinStruct);    

	AdcPinStruct.Pin = Pins.GPIO_Pin_AMP_TEMP_V;            // AMP_TEMP_V
	AdcPinStruct.Mode = GPIO_MODE_ANALOG;
	AdcPinStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Pins.GPIOx_AMP_TEMP_V, &AdcPinStruct);    

	AdcPinStruct.Pin = Pins.GPIO_Pin_AMP_VALVE_TEMP_V;      // VALVE_TEMP_V
	AdcPinStruct.Mode = GPIO_MODE_ANALOG;
	AdcPinStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Pins.GPIOx_AMP_VALVE_TEMP_V, &AdcPinStruct);    
    
	AdcPinStruct.Pin = Pins.GPIO_Pin_V_BATT_SENSE;          // V_BATT_SENSE
	AdcPinStruct.Mode = GPIO_MODE_ANALOG;
	AdcPinStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Pins.GPIOx_AMP_V_BATT_SENSE, &AdcPinStruct);    

    GPIO_InitStruct.Pin = Pins.GPIO_Pin_LED1;               // LED1_N
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(Pins.GPIOx_LED1, &GPIO_InitStruct);    

#ifdef BOARDCONFIG_MK5_MK6 
	AdcPinStruct.Pin = Pins.GPIO_Pin_USB_CC1;               // USB_CC1
	AdcPinStruct.Mode = GPIO_MODE_ANALOG;
	AdcPinStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Pins.GPIOx_USB_CC1, &AdcPinStruct);    
    
	AdcPinStruct.Pin = Pins.GPIO_Pin_USB_CC2;               // USB_CC2
	AdcPinStruct.Mode = GPIO_MODE_ANALOG;
	AdcPinStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Pins.GPIOx_USB_CC2, &AdcPinStruct);    
    
    GPIO_InitStruct.Pin = Pins.GPIO_Pin_LED2;               // LED2_N
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(Pins.GPIOx_LED2, &GPIO_InitStruct);    

#endif

    GPIO_InitStruct.Pin = Pins.GPIO_Pin_AMP_CTRL1;          // AMP_CTRL1
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(Pins.GPIOx_AMP_CTRL1, &GPIO_InitStruct);    

    GPIO_InitStruct.Pin = Pins.GPIO_Pin_AMP_CTRL2;          // AMP_CTRL2
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(Pins.GPIOx_AMP_CTRL2, &GPIO_InitStruct);    

#ifdef BOARDCONFIG_MK5_MK6 
    GPIO_InitStruct.Pin = Pins.GPIO_Pin_VALVE_CTRL1;        // VALVE_CTRL1
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(Pins.GPIOx_VALVE_CTRL1, &GPIO_InitStruct);    

    GPIO_InitStruct.Pin = Pins.GPIO_Pin_VALVE_CTRL2;        // VALVE_CTRL2
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(Pins.GPIOx_VALVE_CTRL2, &GPIO_InitStruct);    
#endif

#ifdef PUSHBUTTON_UI_ENABLED
    GPIO_InitStruct.Pin = Pins.GPIO_Pin_PUSHBUTTON;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Pins.GPIOx_PUSHBUTTON, &GPIO_InitStruct);    
#endif
    pushbutton_value = GPIO_PIN_SET;
    last_pushbutton_value = GPIO_PIN_SET;

    pwm_amp_ctrl.enabled = false;
    pwm_amp_ctrl.suspended = false;
    pwm_amp_ctrl.heater_level_high = false;
    pwm_amp_ctrl.pwm_setting = 0;   
    pwm_amp_ctrl.pwm_state = 0;
    HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL1, Pins.GPIO_Pin_AMP_CTRL1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL2, Pins.GPIO_Pin_AMP_CTRL2, GPIO_PIN_RESET);
            
    pwm_valve_ctrl.enabled = false;
    pwm_valve_ctrl.suspended = false;    
    pwm_valve_ctrl.heater_level_high = false;
    pwm_valve_ctrl.pwm_setting = 0;   
    pwm_valve_ctrl.pwm_state = 0;
#ifdef BOARDCONFIG_MK5_MK6 
    HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL1, Pins.GPIO_Pin_VALVE_CTRL1, GPIO_PIN_RESET);    
    HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL2, Pins.GPIO_Pin_VALVE_CTRL2, GPIO_PIN_RESET);    
#endif

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
    // Control the sample heater control outputs
    // This is aligned with the start of the 125 usec PWM period
    
    if (pwm_amp_ctrl.enabled) {
        if ((TIM1_tick_count & 0xFF)  < pwm_amp_ctrl.pwm_setting) {
            if (pwm_amp_ctrl.heater_level_high) {
                HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL1, Pins.GPIO_Pin_AMP_CTRL1, GPIO_PIN_SET);
                HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL2, Pins.GPIO_Pin_AMP_CTRL2, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL1, Pins.GPIO_Pin_AMP_CTRL1, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL2, Pins.GPIO_Pin_AMP_CTRL2, GPIO_PIN_SET);
            }
            pwm_amp_ctrl.pwm_state++;
        } else {
            HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL1, Pins.GPIO_Pin_AMP_CTRL1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL2, Pins.GPIO_Pin_AMP_CTRL2, GPIO_PIN_RESET);
            pwm_amp_ctrl.pwm_state = 0;
        }
    } else {
        HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL1, Pins.GPIO_Pin_AMP_CTRL1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL2, Pins.GPIO_Pin_AMP_CTRL2, GPIO_PIN_RESET);
        pwm_amp_ctrl.pwm_state = 0;
    }

    // Control the valve heater control outputs
    // This is aligned with the end of the 125 usec PWM period
    if (pwm_valve_ctrl.enabled) {
        if ((TIM1_tick_count & 0xFF)  > (255 - pwm_valve_ctrl.pwm_setting)) {
            if (pwm_amp_ctrl.heater_level_high) {
                HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL1, Pins.GPIO_Pin_VALVE_CTRL1, GPIO_PIN_SET);
                HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL2, Pins.GPIO_Pin_VALVE_CTRL2, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL1, Pins.GPIO_Pin_VALVE_CTRL1, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL2, Pins.GPIO_Pin_VALVE_CTRL2, GPIO_PIN_SET);
            }
            pwm_valve_ctrl.pwm_state++;
        } else {
            HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL1, Pins.GPIO_Pin_VALVE_CTRL1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL2, Pins.GPIO_Pin_VALVE_CTRL2, GPIO_PIN_RESET);
            pwm_valve_ctrl.pwm_state = 0;
        }
    } else {
        HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL1, Pins.GPIO_Pin_VALVE_CTRL1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL2, Pins.GPIO_Pin_VALVE_CTRL2, GPIO_PIN_RESET);
        pwm_valve_ctrl.pwm_state = 0;
    }
        
}

void LEDTimer_ISR(void)
{
    if (data.test_active) {
        HAL_GPIO_TogglePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1);
    } else {
        HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_RESET);
    }
    
    /*
    // blink the LED (at 12.5% duty cycle) if either PWM is enabled:
    if (pwm_amp_ctrl.enabled || pwm_valve_ctrl.enabled) && (TIM1_tick_count & 0x100) < 0x80) {
        if ((TIM1_tick_count & 0x7) == 0) HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_RESET);
        else HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_SET);
    }
    */
    
}
void PIDTimer_ISR(void)
{
    flags.flagUpdatePID = true;
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

void LogData_ISR(void)
{
    flags.flagSendLogData = true;
}

void Pushbutton_ISR(void)
{
    // Latch the pushbutton_flag when the button is pressed (falling edge of PA12)
    pushbutton_value = HAL_GPIO_ReadPin(Pins.GPIOx_PUSHBUTTON, Pins.GPIO_Pin_PUSHBUTTON);
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
