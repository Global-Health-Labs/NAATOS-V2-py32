/*
*   File: main.cpp
*   Project: NAATOS V2
*   Copyright 2025, Global Health Labs
*   Written by: Ryan Calderon, Mike Deeds
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
#define UART_RX_BUFFER_SIZE 128
uint8_t Uart_rxBuffer[UART_RX_BUFFER_SIZE];  // Buffer to store received data
uint16_t Uart_RxSize;

GPIO_InitTypeDef GpioInitStruct;
GPIO_InitTypeDef AdcPinStruct;

Pin_assignments_t Pins;

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


/* Private user code ---------------------------------------------------------*/
void Data_init(void);
void Distribute_PWM_Bits(uint8_t pwm_val, uint64_t *pwm_bit_array);

void start_naat_test(void);
void stop_naat_test(void);

char outputStr[192];

// Heater control structures
Pin_pwm_t pwm_H1_ctrl;
Pin_pwm_t pwm_H2_ctrl; 
#if defined(BOARDCONFIG_MK7R) || defined(BOARDCONFIG_MK7C)
Pin_pwm_t pwm_H3_ctrl;
Pin_pwm_t pwm_H4_ctrl; 
#endif 

#define UID_BASE_ADDR 0x1FFF0E00

// Each PY32 has a unique ID (UID).
void print_UID(void)
{
    //uint8_t *uid;
    
    //uid = (uint8_t *) UID_BASE_ADDR;
    //sprintf(outputStr, "UID_%02X%02X%02X\r\n", uid[15], uid[14], uid[13]);
    //HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000); 
    
#if 0    
    for (int i=0; i<16; i++)
    {
        sprintf(outputStr, "UID %d: %02X\r\n", i, uid[i]);
        HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000); 
    }
#endif
}

// Init PID Controller based on the STATE MACHINE state
void pid_init(heater_t heater, pid_init_t pid_settings){
  float adjusted_setpoint;
  
  if (data.cold_ambient_temp_mode) adjusted_setpoint = pid_settings.setpoint + COLD_TEMP_SETPOINT_OFFSET_C;
  else adjusted_setpoint = pid_settings.setpoint;
  pid_controller_init(
    heater, 
    adjusted_setpoint,
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
    
    print_UID();

    // set up the timers:
    PWMTimerNumber = Register_timer(PWMTimer_ISR,  PWM_TIMER_INTERVAL);
    LEDTimerNumber = Register_timer(LEDTimer_ISR,  LED_TIMER_INTERVAL_STAGE1);   

    // register the DataCollectionTimer before PIDTimer to make sure temperature data is recent.
    DataCollectionTimerNumber = Register_timer(DataCollection_ISR,  DATA_COLLECTION_TIMER_INTERVAL);
    PIDTimerNumber = Register_timer(PIDTimer_ISR,  PID_TIMER_INTERVAL);
    
    MinuteTimerNumber = Register_timer(MinuteTimer_ISR,  MINUTE_TIMER_INTERVAL);
    DelayedStartTimerNumber = Register_timer(DelayedStart_ISR,  STARTUP_DELAY_MS);
#ifdef PUSHBUTTON_UI_ENABLED    
    PushbuttonTimerNumber = Register_timer(Pushbutton_ISR,  PUSHBUTTON_TIMER_INTERVAL);
#else     
    flags.flagPushbutton = true;    // start the test right away
#endif

    LogTimerNumber = Register_timer(LogData_ISR,  LOG_TIMER_INTERVAL);
    
    // INIT PID Structure
    pid_init(H1_HEATER, H1_pid_control[data.state]);
    pid_init(H2_HEATER, H2_pid_control[data.state]);
#if defined(BOARDCONFIG_MK7R) || defined(BOARDCONFIG_MK7C)
    pid_init(H3_HEATER, H3_pid_control[data.state]);		
    pid_init(H4_HEATER, H4_pid_control[data.state]);
#endif
    
    // start the timers:
    Enable_timer(LEDTimerNumber);
    Enable_timer(MinuteTimerNumber);
    
#ifdef PUSHBUTTON_UI_ENABLED    
    Enable_timer(PushbuttonTimerNumber);
#endif
    Enable_timer(DataCollectionTimerNumber);
    
    //HAL_GPIO_WritePin(Pins.GPIOx_LED2, Pins.GPIO_Pin_LED2, GPIO_PIN_RESET); // Turn on LED2    
}

/**
  * @brief  
  * @retval int
  */
int main(void)
{
    //uint32_t start_tick;
    //uint32_t rcc_csr_bootstate;    
    //rcc_csr_bootstate = RCC->CSR;
    
    system_setup();
    
  	//start_tick = TIM1_tick_count;    

#ifdef DEBUG    
    //Distribute_PWM_Bits((uint8_t) 3, (uint64_t *) pwm_H1_ctrl.pwm_bits);    
    //Distribute_PWM_Bits((uint8_t) 7, (uint64_t *) pwm_H1_ctrl.pwm_bits);    
    //Distribute_PWM_Bits((uint8_t) 254, (uint64_t *) pwm_H1_ctrl.pwm_bits);   
#endif

    ADC_Read();
    //sprintf(outputStr, "ADCs: %d %d %d %d %d %d %d\r\n", data.adcReading[0], data.adcReading[1], data.adcReading[6], data.adcReading[4], data.adcReading[5], data.adcReading[7], data.adcReading[8]);
    //HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
    //sprintf(outputStr, "system_input_voltage: %1.2f vcc_mcu_voltage: %1.2f\r\n", data.system_input_voltage, data.vcc_mcu_voltage);
    //HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
    
    // py32_temperature_c does not work correctly. The ADC value reads higher than HAL_ADC_TSCAL2 (85c)
    //sprintf(outputStr, "py32_temperature_c: %1.2f ADC->CHSELR: %08X bits: %d vrefint:%d HAL_ADC_TSCAL1: %d HAL_ADC_TSCAL2: %d\r\n", data.py32_temperature_c, ADC1->CHSELR, data.adcReading[7], data.adcReading[8], HAL_ADC_TSCAL1, HAL_ADC_TSCAL2);
    //HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
    
    data.usb_dn_value = HAL_GPIO_ReadPin(Pins.GPIOx_USB_DN, Pins.GPIO_Pin_USB_DN);
    //sprintf(outputStr, "rcc->csr: %08X usb_cc1_voltage: %1.2f usb_cc2_voltage: %1.2f usb_dn_value: %d\r\n", rcc_csr_bootstate, data.usb_cc1_voltage, data.usb_cc2_voltage, data.usb_dn_value);
    //HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
    //sprintf(outputStr, "H1_temperature_c: %1.2f H2_temperature_c: %1.2f H3_temperature_c: %1.2f H4_temperature_c: %1.2f\r\n", data.H1_temperature_c, data.H2_temperature_c, data.H3_temperature_c, data.H4_temperature_c);
    //HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);   

    data.self_test_H1_start_temp_c = data.H1_temperature_c;
    data.self_test_H2_start_temp_c = data.H2_temperature_c;
#if defined(BOARDCONFIG_MK7R) || defined(BOARDCONFIG_MK7C)
    data.self_test_H3_start_temp_c = data.H3_temperature_c;
    data.self_test_H4_start_temp_c = data.H4_temperature_c;
#endif

    if (Validate_Power_Supply() == false) APP_ErrorHandler(ERR_POWER_SUPPLY);

#if defined(BOARDCONFIG_MK5C) || defined(BOARDCONFIG_MK6C)|| defined(BOARDCONFIG_MK6F) || defined(BOARDCONFIG_MK7C)
    // Verify that the USB power supply can supply at least 1.5A:
    if (Validate_USB_Power_Source() == false) APP_ErrorHandler(ERR_INVALID_USB_POWER_SOURCE);    
#endif
    
    ADC_Set_USB_cc_read_state(false);       // disable reading of USB-C CC voltages in the ADC.
    
    if (data.H1_temperature_c <= COLD_TEMP_OFFSET_THRESHOLD_C) {
        data.cold_ambient_temp_mode = true;
    }

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
            
            // Self-test: The heaters are off. 
            // Throw an error if either heater temperature has risen during the delayed start period.
            if ((data.H1_temperature_c > data.self_test_H1_start_temp_c + SELFTEST_MIN_TEMP_RISE_C/2.0) || (data.H2_temperature_c > data.self_test_H2_start_temp_c + SELFTEST_MIN_TEMP_RISE_C/2.0)) {
                APP_ErrorHandler(ERR_SELFTEST_FAILED); 
            }
            
            // Start the test:
            start_naat_test();                        
            Enable_timer(PWMTimerNumber);            
        }                

        if (flags.flag_1minute) {
            flags.flag_1minute = false;
            data.minute_test_count++;

			if (data.minute_test_count  >= STAGE1_TIME_MIN + STAGE2_TIME_MIN + STAGE3_TIME_MIN + DETECTION_TIME_MIN) {
				// final state -- END
				if (data.state != low_power) {
					data.state = low_power;
					stop_naat_test();

					send_max_temps();
/*
					if (data.H1_max_temperature_c < STAGE1_MIN_VALID_TEMP_C) {
						// Set an alarm if the minimum sample temperature was not reached.
						APP_ErrorHandler(ERR_MIN_STAGE1_TEMP);
					} else if (data.H4_max_temperature_c < STAGE3_MIN_VALID_TEMP_C) {
						// Set an alarm if the minimum valve temperature was not reached.
						APP_ErrorHandler(ERR_MIN_ACTUATION_TEMP);
					} 
*/
				}
			}		
		} else if (data.minute_test_count >= STAGE1_TIME_MIN + STAGE2_TIME_MIN + STAGE3_TIME_MIN) {
			// in detection, shut down the loads
			if (data.state != detection) {
				data.state = detection;
				pid_init(H1_HEATER,H1_pid_control[data.state]);
				pid_init(H2_HEATER,H2_pid_control[data.state]);
				pid_init(H3_HEATER,H3_pid_control[data.state]);
				pid_init(H4_HEATER,H4_pid_control[data.state]);
				pwm_H1_ctrl.enabled = false;      
				pwm_H2_ctrl.enabled = false;      
				pwm_H3_ctrl.enabled = false;      
				pwm_H4_ctrl.enabled = false;      
			}
		} else if (data.minute_test_count >= STAGE1_TIME_MIN + STAGE2_TIME_MIN) {
			if (data.state != stage3) {
				data.state = stage3;                      
				pid_init(H1_HEATER,H1_pid_control[data.state]);
				pid_init(H2_HEATER,H2_pid_control[data.state]);
				pid_init(H3_HEATER,H3_pid_control[data.state]);
				pid_init(H4_HEATER,H4_pid_control[data.state]);
				Update_TimerTickInterval(LEDTimerNumber, LED_TIMER_INTERVAL_ACTIVATION);
			}        
		} else if (data.minute_test_count >= STAGE1_TIME_MIN) {
			if (data.state != stage2) {
				data.state = stage2;
				
				// set valve heater high strength and 100%. (sample heater will be off)
				data.heater_control_not_simultaneous = false;
				pwm_H2_ctrl.heater_level_high = true;   
				
				pid_init(H1_HEATER,H1_pid_control[data.state]);
				pid_init(H2_HEATER,H2_pid_control[data.state]);
				pid_init(H3_HEATER,H3_pid_control[data.state]);
				pid_init(H4_HEATER,H4_pid_control[data.state]);
				Update_TimerTickInterval(LEDTimerNumber, LED_TIMER_INTERVAL_ACTIVATION);
			}
        }

        /*UPDATE INPUT::TEMPERATURE SENSORS*/
        if (flags.flagDataCollection){
            flags.flagDataCollection = false;
            //ADC_data_collect();
        }

        /*UPDATE OUTPUT:HEATER LOAD*/
        if (flags.flagUpdatePID) {
            flags.flagUpdatePID = false;
            if (data.test_active) {
                data.msec_test_count += (PID_TIMER_INTERVAL / TICKS_PER_MSEC);
            }

            ADC_data_collect();
            Update_PID();
        }
        
        if (flags.flagSendLogData) {
            flags.flagSendLogData = false;
            print_log_data();
        }
        if (flags.flagNewUARTData) {
            flags.flagNewUARTData = false;
            Process_UARTRxData();
        }
    }    
}

void start_naat_test(void) {
    data.msec_test_count = 0;
    data.minute_test_count = 0;
    data.state = low_power;
    data.H1_max_temperature_c = 0;
    data.H2_max_temperature_c = 0;
    data.H3_max_temperature_c = 0;
    data.H4_max_temperature_c = 0;
    data.stage1_ramp_time = 0;    

    data.H1_pwm_value = 0;
    data.H2_pwm_value = 0;
    data.H3_pwm_value = 0;
    data.H4_pwm_value = 0;
	
    // Upon system start, a self-test is performed:
    // self_test_1: heaters are turned on in low power mode for up to 10 seconds.
    //      If either heater does not rise in temperature by at least SELFTEST_MIN_TEMP_RISE_C, go to the error state.
    // self_test_2: heaters are turned on in high power mode for up to 10 seconds.
    //      If either heater does not rise in temperature by at least SELFTEST_MIN_TEMP_RISE_C, go to the error state.
    // 
    pwm_H1_ctrl.heater_level_high = false;
    pwm_H2_ctrl.heater_level_high = false;
    data.heater_control_not_simultaneous = false;
    data.self_test_H1_start_temp_c = data.H1_temperature_c;
    data.self_test_H2_start_temp_c = data.H2_temperature_c;
    data.self_test_H3_start_temp_c = data.H3_temperature_c;
    data.self_test_H4_start_temp_c = data.H4_temperature_c;

    //HAL_GPIO_WritePin(Pins.GPIOx_LED2, Pins.GPIO_Pin_LED2, GPIO_PIN_SET); // Turn off LED2    
    
    data.test_active = true;
    if (data.state == low_power) {
#if defined(DEBUG_HEATERS)
        data.state = stage1;     // skip the self test if we are debugging 
        data.heater_control_not_simultaneous = true;
        pwm_H1_ctrl.heater_level_high = false;
        pwm_H2_ctrl.heater_level_high = false;

#elif defined(ENABLE_SELF_TEST)
        data.self_test_H1_start_temp_c = data.H1_temperature_c;
        data.state = self_test_1;
#else
        data.state = stage1;     // skip the self test 
        pid_init(H1_HEATER, H1_pid_control[data.state]);
        pid_init(H2_HEATER, H2_pid_control[data.state]);
        pid_init(H3_HEATER, H3_pid_control[data.state]);
        pid_init(H4_HEATER, H4_pid_control[data.state]);
#endif
    } else {
        APP_ErrorHandler(ERR_FIRMWARE_CONFIG); 	
    }
    pwm_H1_ctrl.enabled = true;    
    pwm_H2_ctrl.enabled = true;
    pwm_H3_ctrl.enabled = true;    
    pwm_H4_ctrl.enabled = true;
      
    Enable_timer(PIDTimerNumber);
    
}

void stop_naat_test(void) {
    data.test_active = false;
    data.state = low_power;
    data.H1_pwm_value = 0;
    data.H2_pwm_value = 0;
    data.H3_pwm_value = 0;
    data.H4_pwm_value = 0;
    
    pwm_H1_ctrl.enabled = false;    
    pwm_H2_ctrl.enabled = false;
    pwm_H3_ctrl.enabled = false;    
    pwm_H4_ctrl.enabled = false;
    
    Disable_timer(LogTimerNumber);                
    Disable_timer(MinuteTimerNumber);
    
	Update_TimerTickInterval(LEDTimerNumber, LED_TIMER_INTERVAL_DONE);
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
    data.self_test_H1_start_temp_c = 0;
    data.self_test_H2_start_temp_c = 0;
    data.self_test_H3_start_temp_c = 0;
    data.self_test_H4_start_temp_c = 0;
    data.cold_ambient_temp_mode = false;
    
    data.H1_pwm_value = 0;
    data.H2_pwm_value = 0;
    data.H3_pwm_value = 0;
    data.H4_pwm_value = 0;
    data.H1_temperature_c = 0;
    data.H2_temperature_c = 0;
    data.H3_temperature_c = 0;
    data.H4_temperature_c = 0;
    data.py32_temperature_c = 0;
    data.msec_tick_count = 0;
    data.msec_test_count = 0;
    data.vcc_mcu_voltage = 0;
    data.system_input_voltage = 0;
    data.H1_max_temperature_c = 0;    
    data.H2_max_temperature_c = 0;
    data.H3_max_temperature_c = 0;    
    data.H4_max_temperature_c = 0;
    data.stage1_ramp_time = 0;    
    data.H1_pwm_during_adc_meas = 0;    
    data.H2_pwm_during_adc_meas = 0;    
    data.H3_pwm_during_adc_meas = 0;    
    data.H4_pwm_during_adc_meas = 0;    
}

void ADC_data_collect(void) 
{
    // Fix the heaters to a known state while reading the ADCs (to minimize measurement error and noise)
    if (pwm_H1_ctrl.enabled) {              
    pwm_H1_ctrl.enabled = false;
    pwm_H1_ctrl.suspended = true;
    if (data.H1_pwm_value > 0) 
        HAL_GPIO_WritePin(Pins.GPIOx_H1_CTRL, Pins.GPIO_Pin_H1_CTRL, GPIO_PIN_SET);
    else 
        HAL_GPIO_WritePin(Pins.GPIOx_H1_CTRL, Pins.GPIO_Pin_H1_CTRL, GPIO_PIN_RESET);
    }

    if (pwm_H2_ctrl.enabled) {              
        pwm_H2_ctrl.enabled = false;      
        pwm_H2_ctrl.suspended = true;
        HAL_GPIO_WritePin(Pins.GPIOx_H2_CTRL, Pins.GPIO_Pin_H2_CTRL, GPIO_PIN_RESET);
    }

    if (pwm_H3_ctrl.enabled) {              
        pwm_H3_ctrl.enabled = false;      
        pwm_H3_ctrl.suspended = true;
        HAL_GPIO_WritePin(Pins.GPIOx_H3_CTRL, Pins.GPIO_Pin_H3_CTRL, GPIO_PIN_RESET);
    }

    if (pwm_H4_ctrl.enabled) {              
        pwm_H4_ctrl.enabled = false;      
        pwm_H4_ctrl.suspended = true;
        HAL_GPIO_WritePin(Pins.GPIOx_H4_CTRL, Pins.GPIO_Pin_H4_CTRL, GPIO_PIN_RESET);
    }

    ADC_Read();

    if (pwm_H1_ctrl.suspended) {
        pwm_H1_ctrl.suspended = false;
        pwm_H1_ctrl.enabled = true;
    }
    if (pwm_H2_ctrl.suspended) {
        pwm_H2_ctrl.suspended = false;
        pwm_H2_ctrl.enabled = true;
    }
    if (pwm_H3_ctrl.suspended) {
        pwm_H3_ctrl.suspended = false;
        pwm_H3_ctrl.enabled = true;
    }
    if (pwm_H4_ctrl.suspended) {
        pwm_H4_ctrl.suspended = false;
        pwm_H4_ctrl.enabled = true;
    }

    // Measure the number of seconds it takes to ramp to the minimum H1 temperature:
    if (data.state == stage3 && data.stage1_ramp_time == 0 && data.H1_temperature_c >= STAGE1_MIN_VALID_TEMP_C) {
        data.stage1_ramp_time = data.msec_test_count / 1000 - (STAGE1_TIME_MIN * 60);
    }
}

void Update_PID(void)
{    
    if (data.state == self_test_1) {
        if (data.H1_temperature_c >= (data.self_test_H1_start_temp_c + SELFTEST_MIN_TEMP_RISE_C)) {
            data.state = self_test_2;
            data.self_test_H2_start_temp_c = data.H2_temperature_c;                   
        } else if (data.msec_test_count > SELFTEST_TIME_MSEC) {
            APP_ErrorHandler(ERR_SELFTEST_FAILED);  
        }
    } else if (data.state == self_test_2) {
        if (data.H2_temperature_c >= (data.self_test_H2_start_temp_c + SELFTEST_MIN_TEMP_RISE_C)) {
            data.state = self_test_3;
            data.self_test_H3_start_temp_c = data.H3_temperature_c;                   
        } else if (data.msec_test_count > SELFTEST_TIME_MSEC * 2) {
            APP_ErrorHandler(ERR_SELFTEST_FAILED);  
        }
    } else if (data.state == self_test_3) {
        if (data.H3_temperature_c >= (data.self_test_H3_start_temp_c + SELFTEST_MIN_TEMP_RISE_C)) {
            data.state = self_test_4;
            data.self_test_H4_start_temp_c = data.H4_temperature_c;                   
        } else if (data.msec_test_count > SELFTEST_TIME_MSEC * 3) {
            APP_ErrorHandler(ERR_SELFTEST_FAILED);  
        }
    } else if (data.state == self_test_4) {
        if (data.H4_temperature_c >= (data.self_test_H4_start_temp_c + SELFTEST_MIN_TEMP_RISE_C)) {
            data.state = stage1;
            pid_init(H1_HEATER,H1_pid_control[data.state]);
            pid_init(H2_HEATER,H2_pid_control[data.state]);
            pid_init(H3_HEATER,H3_pid_control[data.state]);
            pid_init(H4_HEATER,H4_pid_control[data.state]);
        } else if (data.msec_test_count > SELFTEST_TIME_MSEC * 4) {
            APP_ErrorHandler(ERR_SELFTEST_FAILED);  
        }
    } else if (data.state == preheat) {
        if (data.H1_temperature_c >= PREHEAT_TEMP_C && data.H2_temperature_c >= PREHEAT_TEMP_C) {
            data.state = stage1;
#if defined(BOARDCONFIG_MK5AA) || defined(BOARDCONFIG_MK6AA)
            data.heater_control_not_simultaneous = false;
            pwm_H1_ctrl.heater_level_high = false;
            pwm_H2_ctrl.heater_level_high = false;
#elif defined(BOARDCONFIG_MK6C)
            data.heater_control_not_simultaneous = false;
            pwm_H1_ctrl.heater_level_high = true;
            pwm_H2_ctrl.heater_level_high = true;
#else
            data.heater_control_not_simultaneous = false;
            pwm_H1_ctrl.heater_level_high = false;
            pwm_H2_ctrl.heater_level_high = false;
#endif    							
            pid_init(H1_HEATER, H1_pid_control[data.state]);
            pid_init(H2_HEATER, H2_pid_control[data.state]);                                
            pid_init(H3_HEATER, H3_pid_control[data.state]);
            pid_init(H4_HEATER, H4_pid_control[data.state]);                                
            //sprintf(outputStr, "preheat mode ended.\r\n");		   
            //HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
        } else if (data.msec_test_count > PREHEAT_MAX_TIME_MSEC) {
            APP_ErrorHandler(ERR_PREHEAT_TIMEOUT);    
        }
    }
    
    pid_controller_compute(H1_HEATER, data.H1_temperature_c);
    pid_controller_compute(H2_HEATER, data.H2_temperature_c);
    pid_controller_compute(H3_HEATER, data.H3_temperature_c);
    pid_controller_compute(H4_HEATER, data.H4_temperature_c);

    // When heater_control_not_simultaneous is enabled, each heater is run at a fraction of the its PWM value, 
    // based on the HEATER_ELEMENT_POWER_RATIO. Each heater will run at a fraction of its max power output.
    // Since the 2 heater controls are aligned with opposite sides of the PWM period, they will not be
    // turned on simultaniously when in this mode. 
    if (data.heater_control_not_simultaneous) {
#ifdef DEBUG_HEATERS
        // turn on the heaters at full power (no PWM). This should not be done unattended!
        //data.H1_pwm_value = VH_FIXED_PWM_TEST;
        data.H1_pwm_value = 0;       
        data.H2_pwm_value = VH_FIXED_PWM_TEST;
        //data.H2_pwm_value = 0;      
#else        
        // Apply maximum power to each heater during self-test
        data.H1_pwm_value = (PWM_MAX * (100-HEATER_ELEMENT_POWER_RATIO)) /100;
        data.H2_pwm_value = (PWM_MAX * HEATER_ELEMENT_POWER_RATIO) /100;
#endif
    } else {
#ifdef DEBUG_HEATERS
        //data.H1_pwm_value = VH_FIXED_PWM_TEST;
        data.H1_pwm_value = 0;       
        data.H2_pwm_value = VH_FIXED_PWM_TEST;
        //data.H2_pwm_value = 0;       
#else        
        data.H1_pwm_value = pid_data[H1_HEATER].out;
        data.H2_pwm_value = pid_data[H2_HEATER].out;
        data.H3_pwm_value = pid_data[H3_HEATER].out;
        data.H4_pwm_value = pid_data[H4_HEATER].out;
#endif
    }
    
    // Start the PWM controller at a new cycle after updating the PWM values:
    pwm_H1_ctrl.pwm_tick_count = 0;
    pwm_H2_ctrl.pwm_tick_count = 0;
    pwm_H3_ctrl.pwm_tick_count = 0;
    pwm_H4_ctrl.pwm_tick_count = 0;
}

void print_log_data(void) 
{
#if defined(BOARDCONFIG_MK5AA) || defined(BOARDCONFIG_MK6AA) || defined(BOARDCONFIG_MK6F)
    sprintf(outputStr, "%4d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %d %d %d\r\n", data.msec_test_count, data.H1_temperature_c, pid_data[H1_HEATER].setpoint, data.H1_pwm_during_adc_meas, data.H2_temperature_c, pid_data[H2_HEATER].setpoint, data.H2_pwm_during_adc_meas, data.system_input_voltage, data.state, (int) pid_data[H2_HEATER].integrator,(int) pid_data[H2_HEATER].dTerm);
    //sprintf(outputStr, "%4d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %d\r\n", data.msec_test_count, data.H1_temperature_c, pid_data[H1_HEATER].setpoint, data.H1_pwm_during_adc_meas, data.H2_temperature_c, pid_data[H2_HEATER].setpoint, data.H2_pwm_during_adc_meas, data.system_input_voltage, data.state);
#elif defined(BOARDCONFIG_MK7C)
    sprintf(outputStr, "%4d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %d\r\n", data.msec_test_count, data.H1_temperature_c, pid_data[H1_HEATER].setpoint, data.H1_pwm_during_adc_meas, data.H2_temperature_c, pid_data[H2_HEATER].setpoint, data.H2_pwm_during_adc_meas, data.H3_temperature_c, pid_data[H3_HEATER].setpoint, data.H3_pwm_during_adc_meas, data.H4_temperature_c, pid_data[H4_HEATER].setpoint, data.H4_pwm_during_adc_meas, data.vcc_mcu_voltage, data.state);
#elif defined(BOARDCONFIG_MK7R)
    sprintf(outputStr, "%4d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %d\r\n", data.msec_test_count, data.H1_temperature_c, pid_data[H1_HEATER].setpoint, data.H1_pwm_during_adc_meas, data.H2_temperature_c, pid_data[H2_HEATER].setpoint, data.H2_pwm_during_adc_meas, data.H3_temperature_c, pid_data[H3_HEATER].setpoint, data.H3_pwm_during_adc_meas, data.H4_temperature_c, pid_data[H4_HEATER].setpoint, data.H4_pwm_during_adc_meas, data.system_input_voltage, data.state);
#else
    sprintf(outputStr, "%4d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %1.2f, %d %d %d\r\n", data.msec_test_count, data.H1_temperature_c, pid_data[H1_HEATER].setpoint, data.H1_pwm_during_adc_meas, data.H2_temperature_c, pid_data[H2_HEATER].setpoint, data.H2_pwm_during_adc_meas, data.H3_temperature_c, data.H4_temperature_c, data.vcc_mcu_voltage, data.state, (int) pid_data[H2_HEATER].integrator,(int) pid_data[H2_HEATER].dTerm);
    //sprintf(outputStr, "%4d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %d\r\n", data.msec_test_count, data.H1_temperature_c, pid_data[H1_HEATER].setpoint, data.H1_pwm_during_adc_meas, data.H2_temperature_c, pid_data[H2_HEATER].setpoint, data.H1_pwm_during_adc_meas, data.vcc_mcu_voltage, data.state);
#endif
    
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
}

void send_max_temps(void) {
    //sprintf(outputStr, "H1 Max: %1.2f H2 Max: %1.2f H3 Max: %1.2f H4 Max: %1.2f\r\n", data.H1_max_temperature_c, data.H2_max_temperature_c, data.H3_max_temperature_c, data.H4_max_temperature_c);
    //HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
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
    //  PIN 1, PA2 is ADC in: H1_TEMP_V
    //  PIN 2, PA3 is ADC in: H2_TEMP_V
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
    //  PIN 1, PA2 is ADC in: H2_TEMP_V
    //  PIN 2, PA3 is ADC in: H1_TEMP_V
    //  PIN 3, PA4 is ADC in: USB_CC2
    //  PIN 4, PA5 is LED1
    //  PIN 12, PB5 is LED2
    //  PIN 5, PA6 is ADC in: V_BATT_SENSE
    //  PIN 6, PA7 is ADC in: USB_CC1
    //  PIN 13, PB6 is PWM output1: H1_CTRL
    //  PIN 14, PB7 is PWM output2: H2_CTRL
    //  PIN 16, PF0 is PWM output3: H3_CTRL
    //  PIN 17, PF1 is PWM output4: H4_CTRL
    //  PIN 18, PF2 is NRESET pushbutton (not used, pulled up)
    //  PIN 15, PF4 is BOOT0 (not used, pulled down)
    
    // MK7 pins:
	//  PIN 19, PA0 is UART TX
	//  PIN 20, PA1 is H3_TEMP_V
    //  PIN 1, PA2 is ADC in: USB CC1
    //  PIN 2, PA3 is ADC in: USB_CC2
    //  PIN 3, PA4 is ADC in: H4_TEMP_V
    //  PIN 4, PA5 is H1_TEMP_V
    //  PIN 5, PA6 is ADC in: V_BATT_SENSE
    //  PIN 6, PA7 is ADC in: H2_TEMP_V
    //  PIN 8, PA12 is USB_DN
    //  PIN 12, PB5 is LED1
    //  PIN 13, PB6 is PWM output1: H1_CTRL (H1)
    //  PIN 14, PB7 is PWM output2: H2_CTRL (H2)
    //  PIN 16, PF0 is PWM output3: H3_CTRL (H3)
    //  PIN 17, PF1 is PWM output4: H4_CTRL (H4)
    //  PIN 18, PF2 is UART RX / pushbutton in
    //  PIN 15, PF4 is BOOT0 (not used, pulled down)
	
	// Enable peripheral clocks
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
	
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
#if defined(BOARDCONFIG_MK5C) || defined(BOARDCONFIG_MK6C) || defined(BOARDCONFIG_MK5AA) || defined(BOARDCONFIG_MK6AA) || defined(BOARDCONFIG_MK6F)
    Pins.GPIOx_H1_TEMP_V = GPIOA;
    Pins.GPIO_Pin_H1_TEMP_V = GPIO_PIN_2;
    Pins.ADC_CHANNEL_H1_TEMP_V = ADC_CHANNEL_2;    
    Pins.GPIOx_AMP_H2_TEMP_V = GPIOA;
    Pins.GPIO_Pin_AMP_H2_TEMP_V = GPIO_PIN_3;
    Pins.ADC_CHANNEL_H2_TEMP_V = ADC_CHANNEL_3;    
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
    Pins.GPIOx_H1_CTRL = GPIOB;
    Pins.GPIO_Pin_H1_CTRL = GPIO_PIN_6;
    Pins.GPIOx_H2_CTRL = GPIOB;
    Pins.GPIO_Pin_H2_CTRL = GPIO_PIN_7;
    Pins.GPIOx_H3_CTRL = GPIOF;
    Pins.GPIO_Pin_H3_CTRL = GPIO_PIN_0;
    Pins.GPIOx_H4_CTRL = GPIOF;
    Pins.GPIO_Pin_H4_CTRL = GPIO_PIN_1;
    Pins.GPIOx_PUSHBUTTON = GPIOF;
    Pins.GPIO_Pin_PUSHBUTTON = GPIO_PIN_2;
#elif defined(BOARDCONFIG_MK7R) || defined(BOARDCONFIG_MK7C)
    Pins.GPIOx_H1_TEMP_V = GPIOA;
    Pins.GPIO_Pin_H1_TEMP_V = GPIO_PIN_5;
    Pins.ADC_CHANNEL_H1_TEMP_V = ADC_CHANNEL_5;    
    Pins.GPIOx_AMP_H2_TEMP_V = GPIOA;
    Pins.GPIO_Pin_AMP_H2_TEMP_V = GPIO_PIN_7;
    Pins.ADC_CHANNEL_H2_TEMP_V = ADC_CHANNEL_7;    
    Pins.GPIOx_H3_TEMP_V = GPIOA;
    Pins.GPIO_Pin_H3_TEMP_V = GPIO_PIN_1;
    Pins.ADC_CHANNEL_H3_TEMP_V = ADC_CHANNEL_1;    
    Pins.GPIOx_AMP_H4_TEMP_V = GPIOA;
    Pins.GPIO_Pin_AMP_H4_TEMP_V = GPIO_PIN_4;
    Pins.ADC_CHANNEL_H4_TEMP_V = ADC_CHANNEL_4;    
    Pins.GPIOx_AMP_V_BATT_SENSE = GPIOA;
    Pins.GPIO_Pin_V_BATT_SENSE = GPIO_PIN_6;
    Pins.ADC_CHANNEL_V_BATT_SENSE = ADC_CHANNEL_6;    
    Pins.GPIOx_USB_CC1 = GPIOA;
    Pins.GPIO_Pin_USB_CC1 = GPIO_PIN_2;
    Pins.ADC_CHANNEL_USB_CC1 = ADC_CHANNEL_2;    
    Pins.GPIOx_USB_CC2 = GPIOA;
    Pins.GPIO_Pin_USB_CC2 = GPIO_PIN_3;
    Pins.ADC_CHANNEL_USB_CC2 = ADC_CHANNEL_3;    
    Pins.GPIOx_LED1 = GPIOB;
    Pins.GPIO_Pin_LED1 = GPIO_PIN_5;
    Pins.GPIOx_LED2 = GPIOB;
    Pins.GPIO_Pin_LED2 = GPIO_PIN_5;
    Pins.GPIOx_H1_CTRL = GPIOB;
    Pins.GPIO_Pin_H1_CTRL = GPIO_PIN_6;
    Pins.GPIOx_H2_CTRL = GPIOB;
    Pins.GPIO_Pin_H2_CTRL = GPIO_PIN_7;
    Pins.GPIOx_H3_CTRL = GPIOF;
    Pins.GPIO_Pin_H3_CTRL = GPIO_PIN_0;
    Pins.GPIOx_H4_CTRL = GPIOF;
    Pins.GPIO_Pin_H4_CTRL = GPIO_PIN_1;
    Pins.GPIOx_PUSHBUTTON = GPIOF;
    Pins.GPIO_Pin_PUSHBUTTON = GPIO_PIN_2;
    Pins.GPIOx_USB_DN = GPIOA;
    Pins.GPIO_Pin_USB_DN = GPIO_PIN_12;

#else    
    Pins.GPIOx_H1_TEMP_V = GPIOA;
    Pins.GPIO_Pin_H1_TEMP_V = GPIO_PIN_2;
    Pins.ADC_CHANNEL_H1_TEMP_V = ADC_CHANNEL_2;    
    Pins.GPIOx_AMP_H2_TEMP_V = GPIOA;
    Pins.GPIO_Pin_AMP_H2_TEMP_V = GPIO_PIN_3;
    Pins.ADC_CHANNEL_H2_TEMP_V = ADC_CHANNEL_3;    
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
    Pins.GPIOx_H1_CTRL = GPIOB;
    Pins.GPIO_Pin_H1_CTRL = GPIO_PIN_6;
    Pins.GPIOx_H2_CTRL = GPIOB;
    Pins.GPIO_Pin_H2_CTRL = GPIO_PIN_7;
    //Pins.GPIOx_H3_CTRL = GPIOF;
    //Pins.GPIO_Pin_H3_CTRL = GPIO_PIN_0;
    //Pins.GPIOx_H4_CTRL = GPIOF;
    //Pins.GPIO_Pin_H4_CTRL = GPIO_PIN_1;
    Pins.GPIOx_PUSHBUTTON = GPIOA;
    Pins.GPIO_Pin_PUSHBUTTON = GPIO_PIN_12;
#endif
	
	//AdcPinStruct.Pin = GPIO_PIN_1;              // PA1 / UART_RX alternative function
	//AdcPinStruct.Mode = GPIO_MODE_ANALOG;
	//AdcPinStruct.Pull = GPIO_NOPULL;
	//HAL_GPIO_Init(GPIOA, &AdcPinStruct);    

	AdcPinStruct.Pin = Pins.GPIO_Pin_H1_TEMP_V;            // H1_TEMP_V
	AdcPinStruct.Mode = GPIO_MODE_ANALOG;
	AdcPinStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Pins.GPIOx_H1_TEMP_V, &AdcPinStruct);    

	AdcPinStruct.Pin = Pins.GPIO_Pin_AMP_H2_TEMP_V;      // H2_TEMP_V
	AdcPinStruct.Mode = GPIO_MODE_ANALOG;
	AdcPinStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Pins.GPIOx_AMP_H2_TEMP_V, &AdcPinStruct);    
    
	AdcPinStruct.Pin = Pins.GPIO_Pin_V_BATT_SENSE;          // V_BATT_SENSE
	AdcPinStruct.Mode = GPIO_MODE_ANALOG;
	AdcPinStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Pins.GPIOx_AMP_V_BATT_SENSE, &AdcPinStruct);    

    GPIO_InitStruct.Pin = Pins.GPIO_Pin_LED1;               // LED1_N
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(Pins.GPIOx_LED1, &GPIO_InitStruct);    

#if defined(BOARDCONFIG_MK5C) || defined(BOARDCONFIG_MK6C) || defined(BOARDCONFIG_MK5AA) || defined(BOARDCONFIG_MK6AA) || defined(BOARDCONFIG_MK6F)
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
	
#elif  defined(BOARDCONFIG_MK7R) || defined(BOARDCONFIG_MK7C)
	AdcPinStruct.Pin = Pins.GPIO_Pin_H3_TEMP_V;            // H3_TEMP_V
	AdcPinStruct.Mode = GPIO_MODE_ANALOG;
	AdcPinStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Pins.GPIOx_H3_TEMP_V, &AdcPinStruct);    

	AdcPinStruct.Pin = Pins.GPIO_Pin_AMP_H4_TEMP_V;      // H4_TEMP_V
	AdcPinStruct.Mode = GPIO_MODE_ANALOG;
	AdcPinStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Pins.GPIOx_AMP_H4_TEMP_V, &AdcPinStruct);    
    
    AdcPinStruct.Pin = Pins.GPIO_Pin_USB_CC1;               // USB_CC1
	AdcPinStruct.Mode = GPIO_MODE_ANALOG;
	AdcPinStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Pins.GPIOx_USB_CC1, &AdcPinStruct);    
    
	AdcPinStruct.Pin = Pins.GPIO_Pin_USB_CC2;               // USB_CC2
	AdcPinStruct.Mode = GPIO_MODE_ANALOG;
	AdcPinStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Pins.GPIOx_USB_CC2, &AdcPinStruct);    
#endif

    GPIO_InitStruct.Pin = Pins.GPIO_Pin_H1_CTRL;          // H1_CTRL
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Pins.GPIOx_H1_CTRL, &GPIO_InitStruct);    

    GPIO_InitStruct.Pin = Pins.GPIO_Pin_H2_CTRL;          // H2_CTRL
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Pins.GPIOx_H2_CTRL, &GPIO_InitStruct);    

#if defined(BOARDCONFIG_MK5C) || defined(BOARDCONFIG_MK6C) || defined(BOARDCONFIG_MK5AA) || defined(BOARDCONFIG_MK6AA) || defined(BOARDCONFIG_MK6F) || defined(BOARDCONFIG_MK7R) || defined(BOARDCONFIG_MK7C)
    GPIO_InitStruct.Pin = Pins.GPIO_Pin_H3_CTRL;        // H3_CTRL
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Pins.GPIOx_H3_CTRL, &GPIO_InitStruct);    

    GPIO_InitStruct.Pin = Pins.GPIO_Pin_H4_CTRL;        // H4_CTRL
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Pins.GPIOx_H4_CTRL, &GPIO_InitStruct);    
#endif

    GPIO_InitStruct.Pin = Pins.GPIO_Pin_USB_DN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Pins.GPIOx_USB_DN, &GPIO_InitStruct);    

#ifdef PUSHBUTTON_UI_ENABLED
    GPIO_InitStruct.Pin = Pins.GPIO_Pin_PUSHBUTTON;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Pins.GPIOx_PUSHBUTTON, &GPIO_InitStruct);    
#endif
    pushbutton_value = GPIO_PIN_SET;
    last_pushbutton_value = GPIO_PIN_SET;

    pwm_H1_ctrl.enabled = false;
    pwm_H1_ctrl.suspended = false;
    pwm_H1_ctrl.pwm_state = 0;
    pwm_H1_ctrl.pwm_tick_count = 0;
    
    // always start in low power, not_simultaneous mode
    data.heater_control_not_simultaneous = true;    
    pwm_H1_ctrl.heater_level_high = false;
    pwm_H2_ctrl.heater_level_high = false;

    HAL_GPIO_WritePin(Pins.GPIOx_H1_CTRL, Pins.GPIO_Pin_H1_CTRL, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Pins.GPIOx_H2_CTRL, Pins.GPIO_Pin_H2_CTRL, GPIO_PIN_RESET);
            
    pwm_H2_ctrl.enabled = false;
    pwm_H2_ctrl.suspended = false;    
    pwm_H2_ctrl.pwm_state = 0;
    pwm_H2_ctrl.pwm_tick_count = 0;

    pwm_H3_ctrl.enabled = false;
    pwm_H3_ctrl.suspended = false;    
    pwm_H3_ctrl.pwm_state = 0;
    pwm_H3_ctrl.pwm_tick_count = 0;

    pwm_H4_ctrl.enabled = false;
    pwm_H4_ctrl.suspended = false;    
    pwm_H4_ctrl.pwm_state = 0;
    pwm_H4_ctrl.pwm_tick_count = 0;

    HAL_GPIO_WritePin(Pins.GPIOx_H3_CTRL, Pins.GPIO_Pin_H3_CTRL, GPIO_PIN_RESET);    
    HAL_GPIO_WritePin(Pins.GPIOx_H4_CTRL, Pins.GPIO_Pin_H4_CTRL, GPIO_PIN_RESET);    
}

void UART_Init(void)
{
	//Configure UART1
	//PA0 is TX
	//PA1 is RX (PF2 on MK7)
	
  UartHandle.Instance          = USART2;
  UartHandle.Init.BaudRate     = 115200;
  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
	
  if (HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    APP_ErrorHandler(ERR_FIRMWARE_CONFIG);
  }
  
  // Start UART reception:
  // DMA mode will fill the rxBuffer, then call a callback function.
  // Since we don't know how much data is coming in, we will instead detect when the UART rx line is idle.
  // This will call UART_IdleCallback where we will receive the data.
  Uart_RxSize = 0;
  flags.flagNewUARTData = false;
  HAL_UART_Receive_DMA(&UartHandle, Uart_rxBuffer, UART_RX_BUFFER_SIZE);
  
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&UartHandle);
    
    if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&UartHandle); // Clear the idle flag
        UART_IdleCallback(&UartHandle);         // Custom processing
    }
}

void UART_IdleCallback(UART_HandleTypeDef *huart)
{
    // Stop the DMA to get how many bytes were received
    Uart_RxSize = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    HAL_UART_DMAStop(huart);

    // Process the received data in user space
    flags.flagNewUARTData = true;
}

void Process_UARTRxData(void)
{
    sprintf(outputStr, "UartRx: %s\r\n", Uart_rxBuffer);		
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);
    // Restart UART RX DMA reception
    HAL_UART_Receive_DMA(&UartHandle, Uart_rxBuffer, UART_RX_BUFFER_SIZE);
}

// Timer ISR functions are called from the HAL_TIM ISR.
// Only time critical code should be run in these routines.
// It is recommended to set a flag in the ISR that enables lower priority code to run in user space.

void PWMTimer_ISR(void)
{       
    if (data.state == self_test_1) {
        HAL_GPIO_WritePin(Pins.GPIOx_H1_CTRL, Pins.GPIO_Pin_H1_CTRL, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Pins.GPIOx_H2_CTRL, Pins.GPIO_Pin_H2_CTRL, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Pins.GPIOx_H3_CTRL, Pins.GPIO_Pin_H3_CTRL, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Pins.GPIOx_H4_CTRL, Pins.GPIO_Pin_H4_CTRL, GPIO_PIN_RESET);
    } else if (data.state == self_test_2) {
        HAL_GPIO_WritePin(Pins.GPIOx_H1_CTRL, Pins.GPIO_Pin_H1_CTRL, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Pins.GPIOx_H2_CTRL, Pins.GPIO_Pin_H2_CTRL, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Pins.GPIOx_H3_CTRL, Pins.GPIO_Pin_H3_CTRL, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Pins.GPIOx_H4_CTRL, Pins.GPIO_Pin_H4_CTRL, GPIO_PIN_RESET);
    } else if (data.state == self_test_3) {
        HAL_GPIO_WritePin(Pins.GPIOx_H1_CTRL, Pins.GPIO_Pin_H1_CTRL, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Pins.GPIOx_H2_CTRL, Pins.GPIO_Pin_H2_CTRL, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Pins.GPIOx_H3_CTRL, Pins.GPIO_Pin_H3_CTRL, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Pins.GPIOx_H4_CTRL, Pins.GPIO_Pin_H4_CTRL, GPIO_PIN_RESET);
    } else if (data.state == self_test_4) {
        HAL_GPIO_WritePin(Pins.GPIOx_H1_CTRL, Pins.GPIO_Pin_H1_CTRL, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Pins.GPIOx_H2_CTRL, Pins.GPIO_Pin_H2_CTRL, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Pins.GPIOx_H3_CTRL, Pins.GPIO_Pin_H3_CTRL, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Pins.GPIOx_H4_CTRL, Pins.GPIO_Pin_H4_CTRL, GPIO_PIN_SET);
    } else {
        // Control the H1 control outputs
        // This is aligned with the start of the PWM period

        if (pwm_H1_ctrl.enabled && data.H1_pwm_value > 0) {
            if (pwm_H1_ctrl.pwm_tick_count  < data.H1_pwm_value) {
                HAL_GPIO_WritePin(Pins.GPIOx_H1_CTRL, Pins.GPIO_Pin_H1_CTRL, GPIO_PIN_SET);
                pwm_H1_ctrl.pwm_state++;
            } else {
                HAL_GPIO_WritePin(Pins.GPIOx_H1_CTRL, Pins.GPIO_Pin_H1_CTRL, GPIO_PIN_RESET);
            }
        } else {
            HAL_GPIO_WritePin(Pins.GPIOx_H1_CTRL, Pins.GPIO_Pin_H1_CTRL, GPIO_PIN_RESET);
            pwm_H1_ctrl.pwm_state = 0;
        }
        pwm_H1_ctrl.pwm_tick_count += 1;
        if (pwm_H1_ctrl.pwm_tick_count >= PWM_MAX) pwm_H1_ctrl.pwm_tick_count = 0;

        // Control the H2 control outputs
        // This is aligned with the end of the PWM period
        if (pwm_H2_ctrl.enabled && data.H2_pwm_value > 0) {
            if (pwm_H2_ctrl.pwm_tick_count  > (PWM_MAX - data.H2_pwm_value)) {
                HAL_GPIO_WritePin(Pins.GPIOx_H2_CTRL, Pins.GPIO_Pin_H2_CTRL, GPIO_PIN_SET);
                pwm_H2_ctrl.pwm_state++;
            } else {
                HAL_GPIO_WritePin(Pins.GPIOx_H2_CTRL, Pins.GPIO_Pin_H2_CTRL, GPIO_PIN_RESET);
                pwm_H2_ctrl.pwm_state = 0;
            }
        } else {
            HAL_GPIO_WritePin(Pins.GPIOx_H2_CTRL, Pins.GPIO_Pin_H2_CTRL, GPIO_PIN_RESET);
            pwm_H2_ctrl.pwm_state = 0;
        }
        pwm_H2_ctrl.pwm_tick_count += 1;
        if (pwm_H2_ctrl.pwm_tick_count >= PWM_MAX) pwm_H2_ctrl.pwm_tick_count = 0;

        // Control the H3 control outputs
        // This is aligned with the start of the PWM period
        if (pwm_H3_ctrl.enabled && data.H3_pwm_value > 0) {
            if (pwm_H3_ctrl.pwm_tick_count  < data.H3_pwm_value) {
                HAL_GPIO_WritePin(Pins.GPIOx_H3_CTRL, Pins.GPIO_Pin_H3_CTRL, GPIO_PIN_SET);
                pwm_H3_ctrl.pwm_state++;
            } else {
                HAL_GPIO_WritePin(Pins.GPIOx_H3_CTRL, Pins.GPIO_Pin_H3_CTRL, GPIO_PIN_RESET);
            }
        } else {
            HAL_GPIO_WritePin(Pins.GPIOx_H3_CTRL, Pins.GPIO_Pin_H3_CTRL, GPIO_PIN_RESET);
            pwm_H3_ctrl.pwm_state = 0;
        }
        pwm_H3_ctrl.pwm_tick_count += 1;
        if (pwm_H3_ctrl.pwm_tick_count >= PWM_MAX) pwm_H3_ctrl.pwm_tick_count = 0;

        // Control the H4 control outputs
        // This is aligned with the end of the PWM period
        if (pwm_H4_ctrl.enabled && data.H4_pwm_value > 0) {
            if (pwm_H4_ctrl.pwm_tick_count  > (PWM_MAX - data.H4_pwm_value)) {
                HAL_GPIO_WritePin(Pins.GPIOx_H4_CTRL, Pins.GPIO_Pin_H4_CTRL, GPIO_PIN_SET);
                pwm_H4_ctrl.pwm_state++;
            } else {
                HAL_GPIO_WritePin(Pins.GPIOx_H4_CTRL, Pins.GPIO_Pin_H4_CTRL, GPIO_PIN_RESET);
                pwm_H4_ctrl.pwm_state = 0;
            }
        } else {
            HAL_GPIO_WritePin(Pins.GPIOx_H4_CTRL, Pins.GPIO_Pin_H4_CTRL, GPIO_PIN_RESET);
            pwm_H4_ctrl.pwm_state = 0;
        }
        pwm_H4_ctrl.pwm_tick_count += 1;
        if (pwm_H4_ctrl.pwm_tick_count >= PWM_MAX) pwm_H4_ctrl.pwm_tick_count = 0;
    }
}

void LEDTimer_ISR(void)
{
    if (data.state == self_test_1 || data.state == self_test_2 || data.state == preheat) {
        HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_SET); // turn LED off during self-test and preheat
    } else if (data.test_active) {
        HAL_GPIO_TogglePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1);
    } else {
        HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_RESET); // turn LED on at the end of the test
    }    
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

// USB CC1 and CC2 readings: Per the USB standard only one will be valid. (USB cables typically tie either CC1 or CC2 to ground)
// An error will be thrown if neither CC1 nor CC2 reads below 0.75v and 1.75v

#define USB_CC_MIN_V                0.25        // Readings below this indicate that the cable has tied this line to ground
#define USB_CC_MIN_VALID_SOURCE_V   0.75        // Readings between 0.25v and 0.75v are low power supplies (invalid for NAATOS)
#define USB_CC_MAX_VALID_SOURCE_V   2.00        // Readings between 0.75 and 2v are valid USB power supplies
                                                // Readings above 2v are not from a valid USB CC voltage divider 

bool Validate_USB_Power_Source(void)
{
    if (data.usb_cc1_voltage >= USB_CC_MIN_VALID_SOURCE_V && data.usb_cc1_voltage <= USB_CC_MAX_VALID_SOURCE_V) {
        return true;
    }
    else if (data.usb_cc2_voltage >= USB_CC_MIN_VALID_SOURCE_V && data.usb_cc2_voltage <= USB_CC_MAX_VALID_SOURCE_V) {
        return true;
    }
#if defined(BOARDCONFIG_MK7C)
    // allow old legacy USB 2.0 chargers as they should be able to supply at least 1.5A 
    else if (data.usb_dn_value == true) {
        return true;
    }
#endif    
    else return false;
}

bool Validate_Power_Supply(void)
{
    if (data.vcc_mcu_voltage >= VCC_MCU_MIN_VOLTAGE && data.vcc_mcu_voltage <= VCC_MCU_MAX_VOLTAGE) {
        return true;
    }
    else return false;
}

#define LOOP_TIMER_COUNT_100MSEC    0x17000

// This will delay by roughly 100msec * delay_count. It is not very accurate.
void sw_delay_100msec(uint32_t delay_count) {
    for (uint32_t i=0; i < (delay_count * LOOP_TIMER_COUNT_100MSEC); i++);
}

// APP_ErrorHandler will shut down all heaters, then blink the error code number
void APP_ErrorHandler(uint8_t errnum)
{
    __disable_irq(); // Disable all interrupts
    
    // Make sure all heaters are disabled
    HAL_GPIO_WritePin(Pins.GPIOx_H1_CTRL, Pins.GPIO_Pin_H1_CTRL, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Pins.GPIOx_H2_CTRL, Pins.GPIO_Pin_H2_CTRL, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Pins.GPIOx_H3_CTRL, Pins.GPIO_Pin_H3_CTRL, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Pins.GPIOx_H4_CTRL, Pins.GPIO_Pin_H4_CTRL, GPIO_PIN_RESET);
    
    sprintf(outputStr, "APP_ErrorHandler: %d. Halting system.\r\n", errnum);		
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
    
    HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_SET); 
    //HAL_GPIO_WritePin(Pins.GPIOx_LED2, Pins.GPIO_Pin_LED2, GPIO_PIN_RESET); 
    
    while (1)
    {
        HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_RESET);   // LED on for 1 sec
        sw_delay_100msec(10);
        HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_SET);   // LED off for 1 sec        
        sw_delay_100msec(10);

        for (int i = 0; i < errnum; i++) {          // blink the error code
            HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_RESET);   // LED on for 200 msec
            sw_delay_100msec(2);
            HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_SET);   // LED off for 200 msec        
            sw_delay_100msec(2);
        }
        HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_SET);   // LED off for 1 sec        
        sw_delay_100msec(10);
    }
}



/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
