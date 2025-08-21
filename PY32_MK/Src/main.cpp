
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
// Ensure your state_machine.h defines an enum like:
// typedef enum { low_power, self_test_1, self_test_2, preheat, amplification_ramp, amplification, actuation_ramp, actuation, detection } state_t;
#include "alarm.h"
#include "pid.h"
#include "adc.h"
#include "app_data.h"
#include "timers.h"





/* Private define ------------------------------------------------------------*/

//#define DEBUG_HEATERS

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandle;
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
int8_t ActuationDelayTimerNumber;

GPIO_PinState pushbutton_value, last_pushbutton_value;

#define SH_FIXED_PWM_TEST 250
#define VH_FIXED_PWM_TEST 250

CONTROL sample_amp_control[NUMPROCESS] = 
{
  {HEATER_SHUTDOWN_C, 0, 0, 2, 1, .5, false},                                                      // shutdown
  {SAMPLE_ZONE_AMP_RAMP_TARGET_C, 0,0, PID_P_RAMP_TERM, 0, 0, true},                              // AMP RAMP
  {SAMPLE_ZONE_AMP_SOAK_TARGET_C, 0,0, PID_SH_P_TERM, PID_SH_I_TERM, PID_SH_D_TERM, false},        // AMP SOAK
  {SAMPLE_ZONE_VALVE_SOAK_TARGET_C, 0, 0, PID_SH_P_TERM, PID_SH_I_TERM, PID_SH_D_TERM, false},     // ACT SOAK
  {HEATER_SHUTDOWN_C, 0, 0, 2, 5, 1, false},                                                       // DETECTION
  {HEATER_SHUTDOWN_C, 0, 0, 0, 0, 0, false}
};
CONTROL valve_amp_control[NUMPROCESS] = 
{
  {HEATER_SHUTDOWN_C, 0, 0, 0, 0, 0, false},
  {VALVE_ZONE_AMP_RAMP_TARGET_C, 0,0, PID_P_RAMP_TERM, 0, 0, true},
  {VALVE_ZONE_AMP_SOAK_TARGET_C, 0,0, PID_VH_P_TERM, PID_VH_I_TERM, PID_VH_D_TERM, false},
  {VALVE_ZONE_VALVE_SOAK_TARGET_C,0, 0, PID_VH_P_TERM, PID_VH_I_TERM, PID_VH_D_TERM, true},
  {HEATER_SHUTDOWN_C, 0, 0, 0, 0, 0, false},
  {VALVE_ZONE_ACT_RAMP_TARGET_C, 0, 0, PID_P_RAMP_TERM, 0, 0, false}
};



/* Private user code ---------------------------------------------------------*/
void Data_init(void);
void Distribute_PWM_Bits(uint8_t pwm_val, uint64_t *pwm_bit_array);

void start_naat_test(void);
void stop_naat_test(void);

char outputStr[256];

// Heater control structures
Pin_pwm_t pwm_amp_ctrl;
Pin_pwm_t pwm_valve_ctrl; 

#define UID_BASE_ADDR 0x1FFF0E00

int __io_putchar(int ch) {
    HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

// Each PY32 has a unique ID (UID).
void print_UID(void)
{
    uint8_t *uid;
    
    uid = (uint8_t *) UID_BASE_ADDR;
    sprintf(outputStr, "UID_%02X%02X%02X\r\n", uid[15], uid[14], uid[13]);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000); 
    
#if 0    
    for (int i=0; i<16; i++)
    {
        sprintf(outputStr, "UID %d: %02X\r\n", i, uid[i]);
        HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000); 
    }
#endif
}



#define FLAG_ADDRESS  0x0800FC00  // Adjust if using this page for other config
#define FLAG_VALUE    0xDEADBEEF  // Arbitrary "magic" flag

uint32_t ReadFlag(void)
{
    return *(uint32_t *)FLAG_ADDRESS;
}

void WriteFlag(uint32_t value)
{

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError = 0;
    eraseInit.TypeErase   = FLASH_TYPEERASE_PAGEERASE;
    eraseInit.PageAddress = FLAG_ADDRESS;
    eraseInit.NbPages     = 1;

    sprintf(outputStr,"attempting FLASH ERASE");
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);

    if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK) {
        sprintf(outputStr,"Flash erase failed! Error: 0x%lx\r\n", pageError);
        HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000); 
        HAL_FLASH_Lock();
        return;
    }

    sprintf(outputStr,"attempting FLASH PROGRAM");
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000); 

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_PAGE, FLAG_ADDRESS, &value) != HAL_OK) {
        sprintf(outputStr,"Flash program failed!\r\n");
        HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000); 
        HAL_FLASH_Lock();
        return;
    }

    HAL_FLASH_Lock();
    sprintf(outputStr,"Flash flag written: 0x%08lX\r\n", value);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000); 

/*
    HAL_FLASH_Unlock();

    // Erase the page first
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError = 0;

    eraseInit.TypeErase = FLASH_TYPEERASE_PAGEERASE;//FLASH_TYPEERASE_PAGE;
    eraseInit.PageAddress = FLAG_ADDRESS;
    eraseInit.NbPages = 1;

    if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK) {
        // Handle error
        while (1);
    }

    // Write the flag // FLASH_TYPEPROGRAM_WORD
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_PAGE, FLAG_ADDRESS, (uint32_t *)&value) != HAL_OK) {
        // Handle error
        while (1);
    }

    HAL_FLASH_Lock();
    */
}


#ifdef SET_OB_ONCE
void SetOptionBytes(void)
{
    // Unlock FLASH and Option Bytes
    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();

    FLASH_OBProgramInitTypeDef OBInit;
    OBInit.OptionType     = OPTIONBYTE_USER | OPTIONBYTE_RDP;
    OBInit.RDPLevel       = OB_RDP_LEVEL_0;       // 0xAA
    OBInit.USERType       =     OB_USER_BOR_EN | OB_USER_BOR_LEV | OB_USER_NRST_MODE | OB_USER_nBOOT1; //OB_USER_ALL;
    OBInit.USERConfig 		=  
        OB_BOR_ENABLE           |           // Enable BOR
        OB_BOR_LEVEL_1p7_1p8    |           // 1.7V - 1.8V
        OB_RESET_MODE_RESET     |           // GPIO usage for pin OB_RESET_MODE_RESET | OB_RESET_MODE_GPIO
        OB_BOOT1_SYSTEM;                    // Boot1 set to system memory
        //OB_IWDG_SW |                

    if (HAL_FLASH_OBProgram(&OBInit) != HAL_OK) {
        // Error handling
    }
    sprintf(outputStr,"HAL_FLASH_OBProgram done!\r\n");
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000); 

    // Launch option byte loading
    if (HAL_FLASH_OB_Launch() != HAL_OK) {
        // Error handling
    }
    sprintf(outputStr,"HAL_FLASH_OB_Launch() done!\r\n");
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000); 

    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();

}

#endif // SET_OB_ONCE
//IWDG_HandleTypeDef hiwdg;          // Watchdog handle

/*
void InitWatchdog(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 0xFFF;  // Max timeout (around 1s+)
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        printf("Failed to init watchdog!\r\n");
        while (1);
    }
}
*/
void PrintOptionBytes(void)
        {
                FLASH_OBProgramInitTypeDef OBInit;
                HAL_FLASH_OBGetConfig(&OBInit);


                // print 0 pin is GPIO, 1 if it is RESET
                sprintf(outputStr, "Option Bytes:\r\n");
                HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000); 
                sprintf(outputStr, "    RESET pin mode: %s\r\n", (OBInit.USERConfig & OB_RESET_MODE_GPIO) ? "GPIO":"RESET" );
                //printf("    RESET pin mode: %s\r\n", (OBInit.USERConfig & OB_RESET_MODE_RESET) ? "RESET" : "GPIO");
                HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000); 

                printf("Option Bytes:\r\n");
                printf("  RDP Level   : 0x%02X\r\n", OBInit.RDPLevel);
            
                if (OBInit.USERConfig & OB_USER_BOR_EN) {
                    printf("  BOR Level   : ");
                    switch (OBInit.USERConfig & FLASH_OPTR_BOR_LEV) {
                            case OB_BOR_LEVEL_1p7_1p8:  printf("1.8V\r\n"); break;
                            case OB_BOR_LEVEL_1p9_2p0:  printf("1.9V\r\n"); break;
                            case OB_BOR_LEVEL_2p1_2p2:  printf("2.1V\r\n"); break;  // default
                            case OB_BOR_LEVEL_2p3_2p4:  printf("2.3V\r\n"); break;
                            case OB_BOR_LEVEL_2p5_2p6:  printf("2.5V\r\n"); break;
                            case OB_BOR_LEVEL_2p7_2p8:  printf("2.7V\r\n"); break;
                            case OB_BOR_LEVEL_2p9_3p0:  printf("2.9V\r\n"); break;
                            case OB_BOR_LEVEL_3p1_3p2:  printf("3.1V\r\n"); break;
                            default:             printf("Unknown\r\n"); break;
                    }
                }else {
                    printf("BOR disabled\r\n");
                }

                printf("  User Config : 0x%02X\r\n", OBInit.USERConfig);
                printf("    WDG: %s\r\n", (OBInit.USERConfig & OB_IWDG_SW) ? "Software" : "Hardware");
                printf("    STOP Reset: %s\r\n", (OBInit.USERConfig & OB_RESET_MODE_RESET) ? "Disabled" : "Enabled");
                //printf("    STDBY Reset: %s\r\n", (OBInit.USERConfig & OB_STDBY_NO_RST) ? "Disabled" : "Enabled");
        }




// Init PID Controller based on the STATE MACHINE state
void pid_init(heater_t heater, CONTROL pid_settings){
  float adjusted_setpoint;
  
  if (data.cold_ambient_temp_mode && pid_settings.cold_temp_adjusted) adjusted_setpoint = pid_settings.setpoint + COLD_TEMP_SETPOINT_OFFSET_C;
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
    LEDTimerNumber = Register_timer(LEDTimer_ISR,  LED_TIMER_INTERVAL_AMPLIFICATION);   

    // register the DataCollectionTimer before PIDTimer to make sure temperature data is recent.
    DataCollectionTimerNumber = Register_timer(DataCollection_ISR,  DATA_COLLECTION_TIMER_INTERVAL);
    PIDTimerNumber = Register_timer(PIDTimer_ISR,  PID_TIMER_INTERVAL);
    
    MinuteTimerNumber = Register_timer(MinuteTimer_ISR,  MINUTE_TIMER_INTERVAL);
    DelayedStartTimerNumber = Register_timer(DelayedStart_ISR,  STARTUP_DELAY_MS);

    ActuationDelayTimerNumber = Register_timer(ActuationDelay_ISR,  ACTUATION_DELAY_TIMER_INTERVAL);

    // Register the pushbutton timer if enabled, PCB dependent
    #ifdef PUSHBUTTON_UI_ENABLED    
        PushbuttonTimerNumber = Register_timer(Pushbutton_ISR,  PUSHBUTTON_TIMER_INTERVAL);
    #else     
        flags.flagPushbutton = true;    // start the test right away
    #endif

    LogTimerNumber = Register_timer(LogData_ISR,  LOG_TIMER_INTERVAL);
    
    // INIT PID Structure
    pid_init(SAMPLE_HEATER, sample_amp_control[data.state]);
    pid_init(VALVE_HEATER, valve_amp_control[data.state]);
    
    // start the timers:
    Enable_timer(LEDTimerNumber);
    
#ifdef PUSHBUTTON_UI_ENABLED    
    Enable_timer(PushbuttonTimerNumber);
#endif
    Enable_timer(DataCollectionTimerNumber);

        
    HAL_GPIO_WritePin(Pins.GPIOx_LED2, Pins.GPIO_Pin_LED2, GPIO_PIN_RESET); // Turn on LED2    
}

void power_on_tests(void){
    #ifdef ENABLE_POWER_ON_TESTS
        if (Validate_Power_Supply() == false) APP_ErrorHandler(ERR_POWER_SUPPLY);

        #if defined(BOARDCONFIG_MK5C) || defined(BOARDCONFIG_MK6C)|| defined(BOARDCONFIG_MK6F)
            // Verify that the USB power supply can supply at least 1.5A:
            if (Validate_USB_Power_Source() == false) APP_ErrorHandler(ERR_INVALID_USB_POWER_SOURCE);    
        #endif
    #endif
    
    ADC_Set_USB_cc_read_state(false);       // disable reading of USB-C CC voltages in the ADC.
}

void init_adc_data(void){

    ADC_Read();
    //sprintf(outputStr, "ADCs: %d %d %d %d %d %d %d\r\n", data.adcReading[0], data.adcReading[1], data.adcReading[2], data.adcReading[3], data.adcReading[4], data.adcReading[5], data.adcReading[6]);
    //HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
    sprintf(outputStr, "system_input_voltage: %1.2f vcc_mcu_voltage: %1.2f\r\n", data.system_input_voltage, data.vcc_mcu_voltage);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
    
    // py32_temperature_c does not work correctly. The ADC value reads higher than HAL_ADC_TSCAL2 (85c)
    //sprintf(outputStr, "py32_temperature_c: %1.2f ADC->CHSELR: %08X bits: %d vrefint:%d HAL_ADC_TSCAL1: %d HAL_ADC_TSCAL2: %d\r\n", data.py32_temperature_c, ADC1->CHSELR, data.adcReading[5], data.adcReading[6], HAL_ADC_TSCAL1, HAL_ADC_TSCAL2);
    //HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
    
    sprintf(outputStr, "rcc->csr: %08X usb_cc1_voltage: %1.2f usb_cc2_voltage: %1.2f\r\n", rcc_csr_bootstate, data.usb_cc1_voltage, data.usb_cc2_voltage);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
    sprintf(outputStr, "sample_temperature_c: %1.2f valve_temperature_c: %1.2f\r\n", data.sample_temperature_c, data.valve_temperature_c);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);   

    data.self_test_sh_start_temp_c = data.sample_temperature_c;
    data.self_test_vh_start_temp_c = data.valve_temperature_c;
    
}

void APP_UpdateState(void){
/**
 * @brief Updates the PID control and manages the main state machine transitions.
 *
 * This function combines the periodic PID update logic and the primary state machine
 * for the NAATOS application. It should be called at regular intervals (e.g., from a timer interrupt
 * or main loop) to:
 *   - Update test timing and collect ADC data.
 *   - Transition between amplification, actuation ramp, actuation, detection, and low power states
 *     based on elapsed time and temperature conditions.
 *   - Initialize or update PID controllers and PWM outputs as needed for each state.
 *   - Handle actuation delay, ramp targets, and error conditions.
 *   - Shut down loads and trigger alarms if minimum temperature requirements are not met.
 *
 * All state transitions and control actions are performed based on the current values in the
 * global data and flags structures.
*/

/***
 * SELF TEST SECTION
 */
    #ifdef ENABLE_POWER_ON_TESTS
        if (data.state == self_test_1) {
            if (data.sample_temperature_c >= (data.self_test_sh_start_temp_c + SELFTEST_MIN_TEMP_RISE_C)
                && data.valve_temperature_c >= (data.self_test_vh_start_temp_c + SELFTEST_MIN_TEMP_RISE_C)) {
                data.state = self_test_2;
                data.self_test_sh_start_temp_c = data.sample_temperature_c;
                data.self_test_vh_start_temp_c = data.valve_temperature_c;
                pwm_amp_ctrl.heater_level_high = true;
                pwm_valve_ctrl.heater_level_high = true;                        
                //sprintf(outputStr, "Passed self_test_1\r\n");		   
                //HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);
            } else if (data.msec_test_count > SELFTEST_TIME_MSEC) {
                APP_ErrorHandler(ERR_SELFTEST_FAILED);  
            }
        } else if (data.state == self_test_2) {
            if (data.sample_temperature_c >= (data.self_test_sh_start_temp_c + SELFTEST_MIN_TEMP_RISE_C)
                && data.valve_temperature_c >= (data.self_test_vh_start_temp_c + SELFTEST_MIN_TEMP_RISE_C)) {
                data.state = preheat;
                data.sample_temperature_c = data.self_test_sh_start_temp_c;
                data.valve_temperature_c = data.self_test_vh_start_temp_c;
                //sprintf(outputStr, "Passed self_test_2\r\n");		   
                //HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);
            } else if (data.msec_test_count > (2* SELFTEST_TIME_MSEC)) {
                APP_ErrorHandler(ERR_SELFTEST_FAILED);                   
            }
        } else if (data.state == preheat) {
            if (data.sample_temperature_c >= PREHEAT_TEMP_C && data.valve_temperature_c >= PREHEAT_TEMP_C) {
                data.state = amplification_ramp;
                #if defined(BOARDCONFIG_MK5AA) || defined(BOARDCONFIG_MK6AA)
                            data.heater_control_not_simultaneous = false;
                            pwm_amp_ctrl.heater_level_high = false;
                            pwm_valve_ctrl.heater_level_high = false;
                #elif defined(BOARDCONFIG_MK6C)
                            data.heater_control_not_simultaneous = false;
                            pwm_amp_ctrl.heater_level_high = true;
                            pwm_valve_ctrl.heater_level_high = true;
                #else
                            data.heater_control_not_simultaneous = false;
                            pwm_amp_ctrl.heater_level_high = false;
                            pwm_valve_ctrl.heater_level_high = false;
                #endif    							
                pid_init(SAMPLE_HEATER, sample_amp_control[data.state]);
                pid_init(VALVE_HEATER, valve_amp_control[data.state]);                                
                //sprintf(outputStr, "preheat mode ended.\r\n");		   
                //HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
            } else if (data.msec_test_count > PREHEAT_MAX_TIME_MSEC) {
                APP_ErrorHandler(ERR_HEATER_TIMEOUT);    
            }
        } 
    #endif // ENABLE_POWER_ON_TESTS

    /***
     * PRIMARY APP STATE MACHINE
     */
    if (data.state == amplification_ramp) {
        // AMPLIFICATION RAMP

        if ((data.sample_temperature_c >= (SAMPLE_ZONE_AMP_RAMP_TARGET_C - HEATER_RAMP_SETPOINT_OFFSET)) && (data.valve_temperature_c >= (VALVE_ZONE_AMP_RAMP_TARGET_C - HEATER_RAMP_SETPOINT_OFFSET))) {
            // EXCEEDED RAMP TARGET; switch to AMPLIFICATION state
            data.state = amplification;
            data.sample_ramp_time_sec = data.msec_test_count / 1000;
            // initialize the sample and valve heaters to the amplification setpoints:
            pid_init(SAMPLE_HEATER,sample_amp_control[data.state]);
            pid_init(VALVE_HEATER,valve_amp_control[data.state]);

            // RESET the minute test count, APP timer
            #ifdef IGNORE_RAMP_TIME            
                Disable_timer(MinuteTimerNumber);    
                data.minute_test_count = 0;
                Enable_timer(MinuteTimerNumber);        // start the test timer over
            #endif            

        } else if (data.msec_test_count > PREHEAT_MAX_TIME_MSEC) {
            //APP_ErrorHandler(ERR_HEATER_TIMEOUT);    
        }
    }

    if (data.minute_test_count >= AMPLIFICATION_TIME_MIN) {
        if (data.minute_test_count >= AMPLIFICATION_TIME_MIN + ACTUATION_TIME_MIN) {
            // in detection
            if (data.state != detection) { // Ensure 'detection' is a valid enum value from your state machine
                data.state = detection;
                pid_init(SAMPLE_HEATER,sample_amp_control[data.state]);
                pid_init(VALVE_HEATER,valve_amp_control[data.state]);
                pwm_amp_ctrl.enabled = false;      
                pwm_valve_ctrl.enabled = false;      
            }
            if (data.minute_test_count  >= AMPLIFICATION_TIME_MIN + ACTUATION_TIME_MIN + DETECTION_TIME_MIN) {
                // final state -- END
                if (data.state != low_power) {
                    // SHUT DOWN LOADS!!!!
                    data.state = low_power;
                    pid_init(SAMPLE_HEATER,sample_amp_control[data.state]);
                    pid_init(VALVE_HEATER,valve_amp_control[data.state]);
                    
                    Disable_timer(LEDTimerNumber);
                    Update_TimerTickInterval(LEDTimerNumber, LED_TIMER_INTERVAL_DONE);
                    Enable_timer(LEDTimerNumber);

                    stop_naat_test();

                    send_max_temps();
                    if (data.sample_max_temperature_c < AMPLIFICATION_MIN_VALID_TEMP_C) {
                        // Set an alarm if the minimum sample temperature was not reached.
                        //APP_ErrorHandler(ERR_MIN_AMPLIFICATION_TEMP);
                    } else if (data.valve_max_temperature_c < ACTUATION_MIN_VALID_TEMP_C) {
                        // Set an alarm if the minimum valve temperature was not reached.
                        //APP_ErrorHandler(ERR_MIN_ACTUATION_TEMP);
                    } 
                }
            }
        } else if (data.minute_test_count >= AMPLIFICATION_TIME_MIN) {
            // select RAMP or Steady state
            if ( (data.valve_temperature_c >= (VALVE_ZONE_ACT_RAMP_TARGET_C)) && !data.flag_reached_actuation_ramp_target) {
                // Reach the actuation ramp target; start TIMER for actuation delay
                data.flag_reached_actuation_ramp_target = true;
                Enable_timer(ActuationDelayTimerNumber); // start the actuation delay timer
            }

            if (data.state != actuation_ramp && !data.flag_reached_actuation_ramp_target) {
                // Set STATE to ACTUATION RAMP
                data.state = actuation_ramp;
                
                data.valve_ramp_start_time_msec = data.msec_test_count;
                // set valve heater high strength and 100%. (sample heater will be off)
                data.heater_control_not_simultaneous = false;
                pwm_valve_ctrl.heater_level_high = true;   
                
                // set PID values
                pid_init(SAMPLE_HEATER,sample_amp_control[data.state]);
                pid_init(VALVE_HEATER,valve_amp_control[data.state]);
                //Update_TimerTickInterval(LEDTimerNumber, LED_TIMER_INTERVAL_ACTIVATION_RAMP);
            }

            if (flags.flagActuationDelay) {
                if (data.state != actuation) {
                    // Set STATE to ACTUATION
                    data.state = actuation;
                    // disable timer
                    Disable_timer(ActuationDelayTimerNumber);

                    data.valve_ramp_start_time_msec = data.msec_test_count;
                    // set valve heater high strength and 100%. (sample heater will be off)
                    data.heater_control_not_simultaneous = false;
                    pwm_valve_ctrl.heater_level_high = true;   
                    
                    // set PID values
                    pid_init(SAMPLE_HEATER,sample_amp_control[data.state]);
                    pid_init(VALVE_HEATER,valve_amp_control[data.state]);

                    //Update_TimerTickInterval(LEDTimerNumber, LED_TIMER_INTERVAL_ACTIVATION);
                }
            } 
        }
    }
}

/**
 * @brief Main entry point for the application.
 *
 * This function performs the following steps:
 * 1. Reads the boot state from the RCC CSR register.
 * 2. Calls system setup routines.
 * 3. Optionally sets or prints option bytes based on compile-time flags.
 * 4. Initializes ADC data and performs power-on self-tests.
 * 5. Checks for cold ambient temperature and sets mode accordingly.
 * 6. Enters the main application loop, which:
 *    - Updates application state.
 *    - Handles pushbutton events to start/stop tests.
 *    - Manages delayed start logic and self-test error handling.
 *    - Updates minute counters for state transitions.
 *    - Collects ADC data and updates PID control if required.
 *    - Sends log data and performs temperature readings in low power mode.
 *
 * The main loop runs indefinitely, managing application state and responding to hardware events.
 */
int main(void)
{
    //uint32_t start_tick;
    uint32_t rcc_csr_bootstate;
    
    rcc_csr_bootstate = RCC->CSR;
    
    system_setup();

    //InitWatchdog();

    #ifdef SET_FLAG
    if (ReadFlag() != FLAG_VALUE) {
        //SetOptionBytes();
        sprintf(outputStr, "Attempting WriteFlag().\r\n");
        HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000); 
        //WriteFlag(FLAG_VALUE);  // Prevent repeated OB setting
        sprintf(outputStr, "OPTION BYTES SET.  Please manually reset.\r\n");
        HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000); 
        while (1);              // Wait for reset after OB launch
    }
    #endif

    // Set OPTION BYTES; for development
    #if SET_OB_ONCE
            sprintf(outputStr, "made it to SetOptionBytes\r\n");
            HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000); 
            SetOptionBytes();
            sprintf(outputStr, "OPTION BYTES SET.  Please manually reset.\r\n");
            HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000); 
            while (1); // Wait for reset

    #else
        //sprintf(outputStr, "printing OPTION BYTES\r\n");
        //HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000); 
        //PrintOptionBytes();
    
    #endif // SET_OB_ONCE


    // Start setting up for APP

    init_adc_data(); 

    power_on_tests();
    
    
    if (data.sample_temperature_c <= COLD_TEMP_OFFSET_THRESHOLD_C) {
        data.cold_ambient_temp_mode = true;
    }

    while (1) 
    {
    
        // RUN primary application code state LOGIC
        if (data.test_active) {
            APP_UpdateState();
        }

        //HAL_IWDG_Refresh(&hiwdg);

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

            #ifdef ENABLE_POWER_ON_TESTS           
            // Self-test: The heaters are off. 
            // Throw an error if either heater temperature has risen during the delayed start period.
            if ((data.sample_temperature_c > data.self_test_sh_start_temp_c + SELFTEST_MIN_TEMP_RISE_C/2.0) || (data.valve_temperature_c > data.self_test_vh_start_temp_c + SELFTEST_MIN_TEMP_RISE_C/2.0)) {
                APP_ErrorHandler(ERR_SELFTEST_FAILED); 
            }
            #endif            
            
            // Start the test:
            start_naat_test();                        
            Enable_timer(PWMTimerNumber);            
        }                

        // UPDATE MINUTE COUNTER; USED fror state transitions
        if (flags.flag_1minute) {
            flags.flag_1minute = false;
            data.minute_test_count++;
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
            if (data.state == low_power) {            
                ADC_data_collect();         // Read the temperature sensors
            }
            print_log_data();
        }
    }   
}

void start_naat_test(void) {
    data.msec_test_count = 0;
    data.minute_test_count = 0;
    data.state = low_power;
    data.valve_max_temperature_c = 0;
    data.sample_max_temperature_c = 0;
    data.valve_ramp_start_time_msec = 0;        
    data.valve_ramp_time_sec = 0;    
    data.sample_ramp_time_sec = 0;    

    data.sample_heater_pwm_value = 0;
    data.valve_heater_pwm_value = 0;
    
    #ifndef IGNORE_RAMP_TIME        
        Enable_timer(MinuteTimerNumber);    
    #endif	
    // Upon system start, a self-test is performed:
    // self_test_1: heaters are turned on in low power mode for up to 10 seconds.
    //      If either heater does not rise in temperature by at least SELFTEST_MIN_TEMP_RISE_C, go to the error state.
    // self_test_2: heaters are turned on in high power mode for up to 10 seconds.
    //      If either heater does not rise in temperature by at least SELFTEST_MIN_TEMP_RISE_C, go to the error state.
    // 
    data.self_test_sh_start_temp_c = data.sample_temperature_c;
    data.self_test_vh_start_temp_c = data.valve_temperature_c;

    pwm_amp_ctrl.enabled = true;    
    pwm_valve_ctrl.enabled = true;

    HAL_GPIO_WritePin(Pins.GPIOx_LED2, Pins.GPIO_Pin_LED2, GPIO_PIN_SET); // Turn off LED2    
    
    data.test_active = true;

    if (data.state == low_power) {
        // INITIALIZE STATE
        #if defined(DEBUG_HEATERS)
                data.state = amplification;     // skip the self test if we are debugging 
                data.heater_control_not_simultaneous = true;
                pwm_amp_ctrl.heater_level_high = false;
                pwm_valve_ctrl.heater_level_high = false;

        #elif ENABLE_POWER_ON_TESTS
                data.heater_control_not_simultaneous = true;
                pwm_amp_ctrl.heater_level_high = false;
                pwm_valve_ctrl.heater_level_high = false;
                data.state = self_test_1;
        #else         
                data.heater_control_not_simultaneous = false;
                pwm_amp_ctrl.heater_level_high = true;
                pwm_valve_ctrl.heater_level_high = true;
                data.state = amplification_ramp;
        #endif

        // PID init
        pid_init(SAMPLE_HEATER, sample_amp_control[data.state]);
        pid_init(VALVE_HEATER,valve_amp_control[data.state]);

    } else {
        sprintf(outputStr, "Err: Invalid test starting state.\r\n");		   
        HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
    }
    
    // start PID timer
    Enable_timer(PIDTimerNumber);
    
}

void stop_naat_test(void) {
    data.test_active = false;
    data.state = low_power;
    data.sample_heater_pwm_value = 0;
    data.valve_heater_pwm_value = 0;
    
    pwm_amp_ctrl.enabled = false;    
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
    flags.flagDelayedStart = true;
    flags.flagPushbutton = false;
    flags.flag_1minute = false;
    flags.flagActuationDelay = false;
    
    data.test_active = false;
    data.state = low_power;
    data.alarm = no_alarm;
    data.minute_test_count = 0;
    data.self_test_sh_start_temp_c = 0;
    data.self_test_vh_start_temp_c = 0;
    data.cold_ambient_temp_mode = false;
    
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
    data.sample_max_temperature_c = 0;    
    data.sample_ramp_start_time_msec = 0;    
    data.sample_ramp_time_sec = 0;    
    data.valve_ramp_start_time_msec = 0;    
    data.valve_ramp_time_sec = 0;    
    data.sh_pwm_during_adc_meas = 0;    
    data.vh_pwm_during_adc_meas = 0;    
}

void ADC_data_collect(void) 
{
    // Fix the heaters to a known state while reading the ADCs (to minimize measurement error and noise)
    if (pwm_amp_ctrl.enabled) {              
    pwm_amp_ctrl.enabled = false;
    pwm_amp_ctrl.suspended = true;
    HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL1, Pins.GPIO_Pin_AMP_CTRL1, GPIO_PIN_RESET);
    if (data.sample_heater_pwm_value > 0) 
        HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL2, Pins.GPIO_Pin_AMP_CTRL2, GPIO_PIN_SET);
    else 
        HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL2, Pins.GPIO_Pin_AMP_CTRL2, GPIO_PIN_RESET);
    }

    if (pwm_valve_ctrl.enabled) {              
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
    if (data.state == actuation && data.valve_ramp_time_sec == 0 && data.valve_temperature_c >= ACTUATION_MIN_VALID_TEMP_C) {
        data.valve_ramp_time_sec = (data.msec_test_count - data.valve_ramp_start_time_msec) / 1000;
    }
}

void Update_PID(void)
{    
    
    pid_controller_compute(SAMPLE_HEATER, data.sample_temperature_c);
    pid_controller_compute(VALVE_HEATER, data.valve_temperature_c);

    // When heater_control_not_simultaneous is enabled, each heater is run at a fraction of the its PWM value, 
    // based on the HEATER_ELEMENT_POWER_RATIO. Each heater will run at a fraction of its max power output.
    // Since the 2 heater controls are aligned with opposite sides of the PWM period, they will not be
    // turned on simultaniously when in this mode. 
    if (data.heater_control_not_simultaneous) {
        #ifdef DEBUG_HEATERS
                // turn on the heaters at full power (no PWM). This should not be done unattended!
                //data.sample_heater_pwm_value = SH_FIXED_PWM_TEST /2;
                data.sample_heater_pwm_value = 0;       // disable SH
                data.valve_heater_pwm_value = VH_FIXED_PWM_TEST /2;
                //data.valve_heater_pwm_value = 0;       // disable SH
        #else        
        // Distribute maximum power such that heaters are not on simultaneously:
        data.sample_heater_pwm_value = (PWM_MAX * (100-HEATER_ELEMENT_POWER_RATIO)) /100;
        data.valve_heater_pwm_value = (PWM_MAX * HEATER_ELEMENT_POWER_RATIO) /100;
        #endif
    } else {
        #ifdef DEBUG_HEATERS
                //data.sample_heater_pwm_value = SH_FIXED_PWM_TEST;
                data.sample_heater_pwm_value = 0;       // disable SH
                data.valve_heater_pwm_value = VH_FIXED_PWM_TEST;
                //data.valve_heater_pwm_value = 0;       // disable SH
        #else        
                data.sample_heater_pwm_value = pid_data[SAMPLE_HEATER].out;
                data.valve_heater_pwm_value = pid_data[VALVE_HEATER].out;
        #endif
    }
    
    // Start the PWM controller at a new cycle after updating the PWM values:
    pwm_amp_ctrl.pwm_tick_count = 0;
    pwm_valve_ctrl.pwm_tick_count = 0;
}

void print_log_data(void) 
{
#if defined(BOARDCONFIG_MK5AA) || defined(BOARDCONFIG_MK6AA) || defined(BOARDCONFIG_MK6F)
    //sprintf(outputStr, "%4d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %d %d %d\r\n", data.msec_test_count, data.sample_temperature_c, pid_data[SAMPLE_HEATER].setpoint, data.sh_pwm_during_adc_meas, data.valve_temperature_c, pid_data[VALVE_HEATER].setpoint, data.vh_pwm_during_adc_meas, data.system_input_voltage, data.state, (int) pid_data[VALVE_HEATER].integrator,(int) pid_data[VALVE_HEATER].dTerm);
    sprintf(outputStr, "%4d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %d\r\n", data.msec_test_count, data.sample_temperature_c, pid_data[SAMPLE_HEATER].setpoint, data.sh_pwm_during_adc_meas, data.valve_temperature_c, pid_data[VALVE_HEATER].setpoint, data.vh_pwm_during_adc_meas, data.system_input_voltage, data.state);
#else
    //sprintf(outputStr, "%4d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %d %d %d\r\n", data.msec_test_count, data.sample_temperature_c, pid_data[SAMPLE_HEATER].setpoint, data.sh_pwm_during_adc_meas, data.valve_temperature_c, pid_data[VALVE_HEATER].setpoint, data.vh_pwm_during_adc_meas, data.vcc_mcu_voltage, data.state, (int) pid_data[VALVE_HEATER].integrator,(int) pid_data[VALVE_HEATER].dTerm);
    sprintf(outputStr, "%4d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %d\r\n", data.msec_test_count, data.sample_temperature_c, pid_data[SAMPLE_HEATER].setpoint, data.sh_pwm_during_adc_meas, data.valve_temperature_c, pid_data[VALVE_HEATER].setpoint, data.vh_pwm_during_adc_meas, data.vcc_mcu_voltage, data.state);
#endif
    
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
}

void send_max_temps(void) {
    sprintf(outputStr, "SH_MAX: %1.2f VH_MAX: %1.2f sample_ramp_time: %d valve_ramp_time: %d\r\n", data.sample_max_temperature_c, data.valve_max_temperature_c, data.sample_ramp_time_sec, data.valve_ramp_time_sec);
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
    Pins.GPIOx_AMP_TEMP_V = GPIOA;
    Pins.GPIO_Pin_AMP_TEMP_V = GPIO_PIN_2;
    Pins.ADC_CHANNEL_AMP_TEMP_V = ADC_CHANNEL_2;    
    Pins.GPIOx_AMP_VALVE_TEMP_V = GPIOA;
    Pins.GPIO_Pin_AMP_VALVE_TEMP_V = GPIO_PIN_3;
    Pins.ADC_CHANNEL_VALVE_TEMP_V = ADC_CHANNEL_3;    
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

#endif

    GPIO_InitStruct.Pin = Pins.GPIO_Pin_AMP_CTRL1;          // AMP_CTRL1
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Pins.GPIOx_AMP_CTRL1, &GPIO_InitStruct);    

    GPIO_InitStruct.Pin = Pins.GPIO_Pin_AMP_CTRL2;          // AMP_CTRL2
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Pins.GPIOx_AMP_CTRL2, &GPIO_InitStruct);    

#if defined(BOARDCONFIG_MK5C) || defined(BOARDCONFIG_MK6C) || defined(BOARDCONFIG_MK5AA) || defined(BOARDCONFIG_MK6AA) || defined(BOARDCONFIG_MK6F)
    GPIO_InitStruct.Pin = Pins.GPIO_Pin_VALVE_CTRL1;        // VALVE_CTRL1
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Pins.GPIOx_VALVE_CTRL1, &GPIO_InitStruct);    

    GPIO_InitStruct.Pin = Pins.GPIO_Pin_VALVE_CTRL2;        // VALVE_CTRL2
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
    pwm_amp_ctrl.pwm_state = 0;
    pwm_amp_ctrl.pwm_tick_count = 0;
    
    // always start in low power, not_simultaneous mode
    data.heater_control_not_simultaneous = true;    
    pwm_amp_ctrl.heater_level_high = false;
    pwm_valve_ctrl.heater_level_high = false;

    HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL1, Pins.GPIO_Pin_AMP_CTRL1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL2, Pins.GPIO_Pin_AMP_CTRL2, GPIO_PIN_RESET);
            
    pwm_valve_ctrl.enabled = false;
    pwm_valve_ctrl.suspended = false;    
    pwm_valve_ctrl.pwm_state = 0;
    pwm_valve_ctrl.pwm_tick_count = 0;
    HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL1, Pins.GPIO_Pin_VALVE_CTRL1, GPIO_PIN_RESET);    
    HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL2, Pins.GPIO_Pin_VALVE_CTRL2, GPIO_PIN_RESET);    
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
    APP_ErrorHandler(ERR_FIRMWARE_CONFIG);
  }
}

// Timer ISR functions are called from the HAL_TIM ISR.
// Only time critical code should be run in these routines.
// It is recommended to set a flag in the ISR that enables lower priority code to run in user space.

void PWMTimer_ISR(void)
{   
    // Control the sample heater control outputs
    // This is aligned with the start of the PWM period

    #if 0
    HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL1, Pins.GPIO_Pin_AMP_CTRL1, GPIO_PIN_RESET);
    if ((TIM1_tick_count & 0x1) == 0)
        HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL2, Pins.GPIO_Pin_AMP_CTRL2, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL2, Pins.GPIO_Pin_AMP_CTRL2, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL1, Pins.GPIO_Pin_VALVE_CTRL1, GPIO_PIN_SET);
    //HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL2, Pins.GPIO_Pin_VALVE_CTRL2, GPIO_PIN_SET);
    #endif     
    
    if (pwm_amp_ctrl.enabled && data.sample_heater_pwm_value > 0) {
        if (pwm_amp_ctrl.pwm_tick_count  < data.sample_heater_pwm_value) {
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
        }
    } else {
        HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL1, Pins.GPIO_Pin_AMP_CTRL1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL2, Pins.GPIO_Pin_AMP_CTRL2, GPIO_PIN_RESET);
        pwm_amp_ctrl.pwm_state = 0;
    }
    pwm_amp_ctrl.pwm_tick_count += 1;
    if (pwm_amp_ctrl.pwm_tick_count >= PWM_MAX) pwm_amp_ctrl.pwm_tick_count = 0;

    // Control the valve heater control outputs
    // This is aligned with the end of the PWM period
    if (pwm_valve_ctrl.enabled && data.valve_heater_pwm_value > 0) {
        if (pwm_valve_ctrl.pwm_tick_count  > (PWM_MAX - data.valve_heater_pwm_value)) {
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
    pwm_valve_ctrl.pwm_tick_count += 1;
    if (pwm_valve_ctrl.pwm_tick_count >= PWM_MAX) pwm_valve_ctrl.pwm_tick_count = 0;

}

void LEDTimer_ISR(void)
{
    if (data.state == self_test_1 || data.state == self_test_2 || data.state == preheat) {
        HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_SET);       // turn LED off during self-test and preheat
    } else if (data.state == amplification_ramp) {
        HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_RESET);     // turn LED on during amplification_ramp
    } else if (data.state == amplification) {
        HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_SET);       // turn LED off during amplification
    } else if (data.state == actuation || data.state == actuation_ramp) {
        HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_RESET);     // turn LED on during actuation
    } else if (data.state == detection) {
        HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_SET);       // turn LED off during detection
    } else if (data.minute_test_count == 0) {
        HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_RESET);     // turn LED on after system setup completes
    } else {
        HAL_GPIO_TogglePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1);
        //HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_RESET);     // turn LED on at the end of the test
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

void ActuationDelay_ISR(void)
{
    flags.flagActuationDelay = true;
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
    HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL1, Pins.GPIO_Pin_AMP_CTRL1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL2, Pins.GPIO_Pin_AMP_CTRL2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL1, Pins.GPIO_Pin_VALVE_CTRL1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL2, Pins.GPIO_Pin_VALVE_CTRL2, GPIO_PIN_RESET);
    
    sprintf(outputStr, "APP_ErrorHandler: %d. Halting system.\r\n", errnum);		
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
    
    HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_SET); 
    HAL_GPIO_WritePin(Pins.GPIOx_LED2, Pins.GPIO_Pin_LED2, GPIO_PIN_RESET); 
    
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
