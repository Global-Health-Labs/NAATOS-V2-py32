
/*
*   File: main.cpp
*   Project: NAATOS V2
*   Copyright 2025, Global Health Labs
*   Written by: Ryan Calderon, Mike Deeds
*
*   Description:
*   Main application file for NAATOS V2 nucleic acid amplification and detection system.
*   Implements state machine control, PID-based heater management, and test sequencing.
*
*   Heater Control Architecture:
*   ---------------------------
*   The system uses four PWM-controlled heater elements for thermal management:
*   
*   Amplification Heaters (Independent Control):
*     - AMP_CTRL1 (PB6): TIM16_CH1N complementary output
*     - AMP_CTRL2 (PB7): TIM17_CH1N complementary output
*     - Both can operate simultaneously with independent duty cycles
*   
*   Valve Heaters (Multiplexed Control):
*     - VALVE_CTRL1 (PF0): TIM14_CH1 via AF2
*     - VALVE_CTRL2 (PF1): TIM14_CH1 via AF13
*     - Hardware constraint: Only ONE can be active at a time
*     - GPIO alternate function switching routes single timer output to active pin
*   
*   Heater Operating Modes:
*   ----------------------
*   1. High-Side Control (heater_level_high = true):
*      - PWM applied to CH1 (AMP_CTRL1 or VALVE_CTRL1)
*      - CH2 remains off
*      - Used for cold ambient conditions or high-power requirements
*   
*   2. Low-Side Control (heater_level_high = false):
*      - PWM applied to CH2 (AMP_CTRL2 or VALVE_CTRL2)
*      - CH1 remains off
*      - Standard operating mode
*   
*   3. USB Power Limiting Mode (usb_power_limit_mode = true):
*      - Dynamically limits combined PWM to stay within USB current specifications
*      - Proportionally scales both heaters if total exceeds USB_MAX_COMBINED_PWM
*      - Maintains relative power ratio while respecting total power budget
*      - Automatically enabled when system_on_usb_power is detected
*      - Maximum combined PWM: 191 (75% of 255) = ~1.35A @ 5V with typical heaters
*   
*   4. Phase-Shifted Heater Control (heater_phase_shift_mode = true):
*      - Alternates amplification and valve heaters out of phase over time
*      - Reduces peak instantaneous power draw by ~50%
*      - Each heater active for 50% of phase period (default 1 second)
*      - Complements USB power limiting: magnitude + time domain power management
*      - Automatically enabled when system_on_usb_power is detected
*      - Thermal mass smooths temperature variations from duty cycling
*   
*   PID Temperature Control:
*   -----------------------
*   - Two independent PID controllers (sample heater and valve heater)
*   - Setpoints adjusted based on test phase and cold ambient conditions
*   - PWM output (0-255) derived from PID controller output
*   - Update rate: 500ms (configurable via PID_TIMER_INTERVAL)
*   
*   State Machine Overview:
*   ----------------------
*   The system progresses through multiple states during a test cycle:
*   
*   1. low_power        - Heaters off, waiting for test start
*   2. self_test_1/2    - Verify heater functionality (if enabled)
*   3. preheat          - Initial warmup to base temperature
*   4. amplification_ramp - Rapid heating to amplification temperature
*   5. amplification    - Hold temperature for nucleic acid amplification
*   6. actuation_ramp   - Heat valve zone for actuation
*   7. actuation        - Maintain valve temperature for sample transfer
*   8. detection        - Cool down, await result
*   
*   Each state has specific temperature targets and PID parameters defined
*   in sample_amp_control[] and valve_amp_control[] arrays.
*   
*   For detailed PWM implementation, see:
*   - Src/io/pwm_init.c - PWM timer configuration and GPIO multiplexing
*   - Inc/io/pwm_init.h - Public API documentation
*   - PWM_ARCHITECTURE.md - Comprehensive architecture documentation
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

#include "state_machine.h"
// Ensure your state_machine.h defines an enum like:
// typedef enum { low_power, self_test_1, self_test_2, preheat, amplification_ramp, amplification, actuation_ramp, actuation, detection } state_t;
#include "alarm.h"
extern "C" {
#include "pid.h"
}
#include "adc.h"
#include "app_data.h"
#include "timers.h"
#include "error_handler.h"

#include "io/gpio_init.h"
#include "io/uart_init.h"
#include "io/adc_init.h"
#include "io/pwm_init.h"


#define USB_MIN_VALID_SOURCE_V      4.5         // Minimum USB power supply voltage for NAATOS operation
#define USB_CC_MIN_V                0.25        // Readings below this indicate that the cable has tied this line to ground
#define USB_CC_MIN_VALID_SOURCE_V   0.75        // Readings between 0.25v and 0.75v are low power supplies (invalid for NAATOS)
#define USB_CC_MAX_VALID_SOURCE_V   2.00        // Readings between 0.75 and 2v are valid USB power supplies
                                                // Readings above 2v are not from a valid USB CC voltage divider 

//#define DEBUG_HEATERS

/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GpioInitStruct;
GPIO_InitTypeDef AdcPinStruct;



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


CONTROL sample_amp_control[NUM_STATES] = 
{
  {HEATER_SHUTDOWN_C, 0, 0, 2, 1, .5, false,0},                                                      // shutdown
  {SAMPLE_ZONE_AMP_RAMP_TARGET_C, 0,0, PID_P_RAMP_TERM, 0, 0, true,-12},                             // AMP RAMP
  {SAMPLE_ZONE_AMP_SOAK_TARGET_C, 0,0, PID_SH_P_TERM, PID_SH_I_TERM, PID_SH_D_TERM, false,0},        // AMP SOAK
  {SAMPLE_ZONE_VALVE_SOAK_TARGET_C, 0, 0, PID_SH_P_TERM, PID_SH_I_TERM, PID_SH_D_TERM, false,0},     // ACT SOAK
  {HEATER_SHUTDOWN_C, 0, 0, 2, 5, 1, false,0},                                                       // DETECTION
  {HEATER_SHUTDOWN_C, 0, 0, 0, 0, 0, false,0}                                                        // ACT RAMP 
};
CONTROL valve_amp_control[NUM_STATES] = 
{
  {HEATER_SHUTDOWN_C, 0, 0, 0, 0, 0, false,0},                                                       // shutdown
  {VALVE_ZONE_AMP_RAMP_TARGET_C, 0,0, PID_P_RAMP_TERM, 0, 0, false,0},                               // AMP RAMP
  {VALVE_ZONE_AMP_SOAK_TARGET_C, 0,0, PID_VH_P_TERM, PID_VH_I_TERM, PID_VH_D_TERM, false,0},         // AMP SOAK
  {VALVE_ZONE_VALVE_SOAK_TARGET_C,0, 0, PID_VH_P_TERM, PID_VH_I_TERM, PID_VH_D_TERM, false,0},       // ACT SOAK
  {HEATER_SHUTDOWN_C, 0, 0, 0, 0, 0, false,0},                                                       // DETECTION
  {VALVE_ZONE_ACT_RAMP_TARGET_C, 0, 0, PID_P_RAMP_TERM, 0, 0, false,0}                               // ACT RAMP
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
/**
 * @brief Prints the unique device ID (UID) over UART.
 */
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

/**
 * @brief Reads a flag value from flash memory.
 * @return The flag value as a uint32_t.
 */
uint32_t ReadFlag(void)
{
    return *(uint32_t *)FLAG_ADDRESS;
}

/**
 * @brief Writes a flag value to flash memory.
 * @param value The value to write.
 */
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
/**
 * @brief Sets option bytes for the MCU (if SET_OB_ONCE is defined).
 */
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
/**
 * @brief Prints the current option byte configuration to UART.
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
                printf("  RDP Level   : 0x%02lX\r\n", (unsigned long)OBInit.RDPLevel);
            
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

                printf("  User Config : 0x%02lX\r\n", (unsigned long)OBInit.USERConfig); 
                printf("    WDG: %s\r\n", (OBInit.USERConfig & OB_IWDG_SW) ? "Software" : "Hardware");
                printf("    STOP Reset: %s\r\n", (OBInit.USERConfig & OB_RESET_MODE_RESET) ? "Disabled" : "Enabled");
                //printf("    STDBY Reset: %s\r\n", (OBInit.USERConfig & OB_STDBY_NO_RST) ? "Disabled" : "Enabled");
        }




/**
 * @brief Initializes the PID controller for a given heater and settings.
 * @param heater The heater to initialize.
 * @param pid_settings The PID settings to use.
 */
void pid_init(heater_t heater, CONTROL pid_settings){
  float adjusted_setpoint;
  if (data.cold_ambient_temp_mode && pid_settings.cold_temp_adjusted) {
    adjusted_setpoint = pid_settings.setpoint + pid_settings.cold_temp_offset_c;
  } else {
    adjusted_setpoint = pid_settings.setpoint;
  }

  pid_controller_init(
    heater, 
    adjusted_setpoint,
    pid_settings.kp,
    pid_settings.ki,
    pid_settings.kd,
    PWM_MAX,
    SLEW_RATE_LIMIT);
}

/**
 * @brief Initializes system peripherals, timers, and PID controllers.
 */
void system_setup() {
    HAL_Init();
  
    APP_SystemClockConfig(); 
    
    //Initialize peripherals
    GPIO_Init();
    TIMER_Init();
    UART_Init();
    ADC_Init();
    PWM_Init();
    Data_init();

    pushbutton_value = GPIO_PIN_SET;
    last_pushbutton_value = GPIO_PIN_SET;

    // Initialize PWM control structures
    pwm_amp_ctrl.enabled = false;
    pwm_valve_ctrl.enabled = false;
    
    // Start with both heaters on low-side control
    pwm_amp_ctrl.heater_level_high = false;
    pwm_valve_ctrl.heater_level_high = false;

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

/**
 * @brief Performs power-on self-tests and validates power supply.
 */
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

/**
 * @brief Initializes ADC data and determines power source.
 */
void init_adc_data(void){

    //uint32_t start_tick;
    uint32_t rcc_csr_bootstate;
    
    rcc_csr_bootstate = RCC->CSR;

    ADC_Read();
    if (data.system_input_voltage > USB_MIN_VALID_SOURCE_V) {
        data.system_on_usb_power = true;
        
        #if USB_POWER_LIMIT_ENABLED
        // Enable USB power limiting when running on USB power
        data.usb_power_limit_mode = true;
        sprintf(outputStr, "USB power limiting enabled (max combined PWM: %d)\r\n", USB_MAX_COMBINED_PWM);
        HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);
        #endif
        
        #if HEATER_PHASE_SHIFT_ENABLED
        // Enable phase-shifted heater control when running on USB power
        // This further reduces peak power draw by alternating heaters over time
        data.heater_phase_shift_mode = true;
        sprintf(outputStr, "Phase-shift control enabled (period: %dms, duty: %d%%)\r\n", 
                HEATER_PHASE_PERIOD_MS, HEATER_PHASE_DUTY_PERCENT);
        HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);
        #endif
    }

    //sprintf(outputStr, "ADCs: %d %d %d %d %d %d %d\r\n", data.adcReading[0], data.adcReading[1], data.adcReading[2], data.adcReading[3], data.adcReading[4], data.adcReading[5], data.adcReading[6]);
    //HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
    sprintf(outputStr, "system_input_voltage: %1.2f vcc_mcu_voltage: %1.2f\r\n", data.system_input_voltage, data.vcc_mcu_voltage);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
    
    // py32_temperature_c does not work correctly. The ADC value reads higher than HAL_ADC_TSCAL2 (85c)
    //sprintf(outputStr, "py32_temperature_c: %1.2f ADC->CHSELR: %08X bits: %d vrefint:%d HAL_ADC_TSCAL1: %d HAL_ADC_TSCAL2: %d\r\n", data.py32_temperature_c, ADC1->CHSELR, data.adcReading[5], data.adcReading[6], HAL_ADC_TSCAL1, HAL_ADC_TSCAL2);
    //HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
    
    sprintf(outputStr, "rcc->csr: %08lX usb_cc1_voltage: %1.2f usb_cc2_voltage: %1.2f\r\n", (unsigned long)rcc_csr_bootstate, data.usb_cc1_voltage, data.usb_cc2_voltage);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
    sprintf(outputStr, "sample_temperature_c: %1.2f valve_temperature_c: %1.2f\r\n", data.sample_temperature_c, data.valve_temperature_c);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);   

    data.self_test_sh_start_temp_c = data.sample_temperature_c;
    data.self_test_vh_start_temp_c = data.valve_temperature_c;
    
}

/**
 * @brief Updates the PID control and manages the main state machine transitions.
 * This function combines the periodic PID update logic and the primary state machine
 * for the NAATOS application. It should be called at regular intervals (e.g., from a timer interrupt
 * or main loop) to:
 *   - Update test timing and collect ADC data.
 *   - Transition between amplification, actuation ramp, actuation, detection, and low power states
 *     based on elapsed time and temperature conditions.
 *   - Initialize or update PID controllers and PWM outputs as needed for each state.
 *   - Handle actuation delay, ramp targets, and error conditions.
 *   - Shut down loads and trigger alarms if minimum temperature requirements are not met.
 * All state transitions and control actions are performed based on the current values in the
 * global data and flags structures.
 */
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
                            pwm_amp_ctrl.heater_level_high = false;
                            pwm_valve_ctrl.heater_level_high = false;
                #elif defined(BOARDCONFIG_MK6C)
                            pwm_amp_ctrl.heater_level_high = true;
                            pwm_valve_ctrl.heater_level_high = true;
                #else
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

        if ((data.sample_max_temperature_c >= (pid_data[SAMPLE_HEATER].setpoint - HEATER_RAMP_SETPOINT_OFFSET)) && (data.valve_max_temperature_c >= (pid_data[VALVE_HEATER].setpoint - HEATER_RAMP_SETPOINT_OFFSET))) {
            // reached the ramp target at least once; set the flag
        //    data.flag_reached_amp_ramp_target = true;
        //}
        //if ((data.sample_temperature_c >= (SAMPLE_ZONE_AMP_RAMP_TARGET_C - HEATER_RAMP_SETPOINT_OFFSET)) && (data.valve_temperature_c >= (VALVE_ZONE_AMP_RAMP_TARGET_C - HEATER_RAMP_SETPOINT_OFFSET))) {
            // EXCEEDED RAMP TARGET; switch to AMPLIFICATION state
            data.state = amplification;
            data.sample_ramp_time_sec = data.msec_test_count / 1000;

            // initialize the sample and valve heaters to the amplification setpoints:
            pid_init(SAMPLE_HEATER,sample_amp_control[data.state]);
            pid_init(VALVE_HEATER,valve_amp_control[data.state]);
            
            // control heater levels
            pwm_amp_ctrl.heater_level_high = false;
            pwm_valve_ctrl.heater_level_high = false;   // mk8

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
            if (data.state != actuation_ramp && !data.flag_reached_actuation_ramp_target) {
                // Set STATE to ACTUATION RAMP
                data.state = actuation_ramp;
                
                data.valve_ramp_start_time_msec = data.msec_test_count;
                // set valve heater high strength and 100%. (sample heater will be off)
                pwm_amp_ctrl.heater_level_high = false;
                pwm_valve_ctrl.heater_level_high = true;   // mk8
                //pwm_valve_ctrl.heater_level_high = true;   
                
                // set PID values
                pid_init(SAMPLE_HEATER,sample_amp_control[data.state]);
                pid_init(VALVE_HEATER,valve_amp_control[data.state]);
                //Update_TimerTickInterval(LEDTimerNumber, LED_TIMER_INTERVAL_ACTIVATION_RAMP);
            }

            if ( (data.valve_temperature_c >= (pid_data[VALVE_HEATER].setpoint)) && !data.flag_reached_actuation_ramp_target) {
                // Reach the actuation ramp target; start TIMER for actuation delay
                data.flag_reached_actuation_ramp_target = true;
                Enable_timer(ActuationDelayTimerNumber); // start the actuation delay timer
            }


            if (flags.flagActuationDelay) {
                if (data.state != actuation) {
                    // Set STATE to ACTUATION
                    data.state = actuation;
                    // disable timer
                    Disable_timer(ActuationDelayTimerNumber);

                    data.valve_ramp_start_time_msec = data.msec_test_count;
                    // set valve heater high strength and 100%. (sample heater will be off)
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

/**
 * @brief Starts a NAAT test, initializes state and PID controllers.
 */
void start_naat_test(void) {
    // Reset test counters and states
    data.msec_test_count = 0;
    data.minute_test_count = 0;
    data.state = low_power;
    data.valve_max_temperature_c = 0;
    data.sample_max_temperature_c = 0;
    data.valve_ramp_start_time_msec = 0;        
    data.valve_ramp_time_sec = 0;    
    data.sample_ramp_time_sec = 0;    

    // Reset PWM values
    data.sample_heater_pwm_value = 0;
    data.valve_heater_pwm_value = 0;
    
    // Enable minute timer if not ignoring ramp time
    #ifndef IGNORE_RAMP_TIME        
        Enable_timer(MinuteTimerNumber);    
    #endif	

    // Store initial temperatures for self-test
    data.self_test_sh_start_temp_c = data.sample_temperature_c;
    data.self_test_vh_start_temp_c = data.valve_temperature_c;

    // Enable PWM outputs
    pwm_amp_ctrl.enabled = true;    
    pwm_valve_ctrl.enabled = true;
    
    // Initialize PWM values to 0
    PWM_Set_Sample_Heater_Channels(0, 0);
    PWM_Set_Valve_Heater_Channels(0, 0);

    // Set LED2 state
    HAL_GPIO_WritePin(Pins.GPIOx_LED2, Pins.GPIO_Pin_LED2, GPIO_PIN_SET); // Turn off LED2    
    
    data.test_active = true;

    if (data.state == low_power) {
        // Set initial state based on build configuration
        #if defined(DEBUG_HEATERS)
                data.state = amplification;     // Skip self-test in debug mode
                //data.heater_phase_shift_mode = false;
                pwm_amp_ctrl.heater_level_high = false;
                pwm_valve_ctrl.heater_level_high = false;

        #elif ENABLE_POWER_ON_TESTS
                //data.heater_phase_shift_mode = false;
                pwm_amp_ctrl.heater_level_high = false;
                pwm_valve_ctrl.heater_level_high = false;
                data.state = self_test_1;
        #else         
                /*
                if (data.system_on_usb_power) {
                    data.heater_phase_shift_mode = true;
                }else {
                    data.heater_phase_shift_mode = false;
                }
                */
                // Default to low-side control
                pwm_amp_ctrl.heater_level_high = false;
                pwm_valve_ctrl.heater_level_high = false;

                // Use high-side control in cold ambient mode
                if (data.cold_ambient_temp_mode) {
                    pwm_amp_ctrl.heater_level_high = true;
                    pwm_valve_ctrl.heater_level_high = true;
                }
                data.state = amplification_ramp;
        #endif

        // Initialize PID controllers
        pid_init(SAMPLE_HEATER, sample_amp_control[data.state]);
        pid_init(VALVE_HEATER, valve_amp_control[data.state]);

    } else {
        sprintf(outputStr, "Err: Invalid test starting state.\r\n");		   
        HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
    }
    
    // Start PID control timer
    Enable_timer(PIDTimerNumber);
}

/**
 * @brief Stops the NAAT test and disables timers and PWM outputs.
 */
void stop_naat_test(void) {
    // Reset test state
    data.test_active = false;
    data.state = low_power;
    
    // Clear PWM values
    data.sample_heater_pwm_value = 0;
    data.valve_heater_pwm_value = 0;
    
    // Disable PWM outputs
    pwm_amp_ctrl.enabled = false;    
    pwm_valve_ctrl.enabled = false;
    
    // Set all PWM channels to 0
    PWM_Set_Sample_Heater_Channels(0, 0);
    PWM_Set_Valve_Heater_Channels(0, 0);
    
    // Disable timers
    Disable_timer(LogTimerNumber);                
    Disable_timer(MinuteTimerNumber);
    
    // Notify user
    sprintf(outputStr, "Test stopped.\r\n");		
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
}

/**
 * @brief Initializes the data and flags structures to default values.
 */
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
    data.usb_power_limit_mode = false;  // Will be set based on power source detection
    data.heater_phase_shift_mode = false;  // Will be enabled based on power source
    data.heater_phase_counter_ms = 0;  // Initialize phase tracking counter
    
    data.sample_heater_pwm_value = 0;
    data.valve_heater_pwm_value = 0;
    data.sample_temperature_c = 0;
    data.valve_temperature_c = 0;
    data.py32_temperature_c = 0;
    data.msec_tick_count = 0;
    data.msec_test_count = 0;
    data.vcc_mcu_voltage = 0;
    data.system_input_voltage = 0;
    data.system_on_usb_power = false;
    data.valve_max_temperature_c = 0;
    data.sample_max_temperature_c = 0;    
    data.sample_ramp_start_time_msec = 0;    
    data.sample_ramp_time_sec = 0;    
    data.valve_ramp_start_time_msec = 0;    
    data.valve_ramp_time_sec = 0;    
    data.sh_pwm_during_adc_meas = 0;    
    data.vh_pwm_during_adc_meas = 0;    
}

/**
 * @brief Collects ADC data and manages heater control during measurement.
 */
void ADC_data_collect(void) 
{
    // Stop PWM outputs before reading ADCs to minimize measurement error and noise
    if (pwm_amp_ctrl.enabled) {              
        pwm_amp_ctrl.enabled = false;
        pwm_amp_ctrl.suspended = true;
        PWM_Set_Sample_Heater_Channels(0, 0);
    }

    if (pwm_valve_ctrl.enabled) {              
        pwm_valve_ctrl.enabled = false;      
        pwm_valve_ctrl.suspended = true;
        PWM_Set_Valve_Heater_Channels(0, 0);
    }

    ADC_Read();

    // Restore PWM outputs 
    if (pwm_amp_ctrl.suspended) {
        pwm_amp_ctrl.suspended = false;
        pwm_amp_ctrl.enabled = true;
        // PWM values will be updated in next Update_PID() call
    }
    if (pwm_valve_ctrl.suspended) {
        pwm_valve_ctrl.suspended = false;
        pwm_valve_ctrl.enabled = true;
        // PWM values will be updated in next Update_PID() call
    }

    // Measure the number of seconds it takes to ramp to the minimum valve temperature:
    if (data.state == actuation && data.valve_ramp_time_sec == 0 && data.valve_temperature_c >= ACTUATION_MIN_VALID_TEMP_C) {
        data.valve_ramp_time_sec = (data.msec_test_count - data.valve_ramp_start_time_msec) / 1000;
    }
}

/**
 * @brief Apply USB power budget limiting to heater PWM values
 * 
 * When operating on USB power (current limited), this function ensures the
 * combined PWM duty cycle of both heaters stays within safe limits to prevent
 * exceeding the USB current specification.
 * 
 * Power Limiting Strategy:
 * -----------------------
 * 1. Calculate total requested PWM (sample + valve)
 * 2. If total exceeds USB_MAX_COMBINED_PWM:
 *    a. Scale both heaters down proportionally
 *    b. Maintain relative power ratio between heaters
 *    c. Keep total at or below USB_MAX_COMBINED_PWM
 * 
 * USB Current Budgets:
 * -------------------
 * - USB 2.0:      500mA max (USB_MAX_COMBINED_PWM = 191 = 75%)
 * - USB 3.0:      900mA max (could use higher limit)
 * - USB-C PD:     1.5A-3A max (negotiated, could disable limiting)
 * 
 * Example Scaling:
 * ---------------
 * If sample PWM = 200 and valve PWM = 150:
 *   Total = 350 (exceeds 191 limit)
 *   Scale factor = 191 / 350 = 0.546
 *   New sample PWM = 200 * 0.546 = 109
 *   New valve PWM = 150 * 0.546 = 82
 *   New total = 191 (within limit)
 * 
 * @note This function modifies data.sample_heater_pwm_value and data.valve_heater_pwm_value
 * @note Only active when data.usb_power_limit_mode is true
 * @note PID controllers compute target values; this function limits actual output
 * @note Proportional scaling maintains temperature control characteristics
 * 
 * @warning Excessive power limiting may cause inability to reach target temperatures
 * @warning Application should adjust setpoints if continuous limiting occurs
 */
static void Apply_USB_Power_Limiting(void)
{
#if USB_POWER_LIMIT_ENABLED
    // Only apply limiting when in USB power limit mode
    if (!data.usb_power_limit_mode) {
        return;
    }
    
    // Calculate total requested PWM
    uint16_t total_pwm = data.sample_heater_pwm_value + data.valve_heater_pwm_value;
    
    // Check if total exceeds the USB power budget
    if (total_pwm > USB_MAX_COMBINED_PWM) {
        // Calculate scaling factor to bring total within limit
        // Use integer math with fixed-point to avoid float operations in time-critical code
        // Scale factor = (USB_MAX_COMBINED_PWM * 1000) / total_pwm
        uint32_t scale_factor = ((uint32_t)USB_MAX_COMBINED_PWM * 1000) / total_pwm;
        
        // Apply proportional scaling to both heaters
        // Maintains relative power ratio while respecting total power budget
        data.sample_heater_pwm_value = (data.sample_heater_pwm_value * scale_factor) / 1000;
        data.valve_heater_pwm_value = (data.valve_heater_pwm_value * scale_factor) / 1000;
        
        // Verify we're within limits (should always be true with correct math)
        uint16_t new_total = data.sample_heater_pwm_value + data.valve_heater_pwm_value;
        
        // If due to rounding we're still slightly over, reduce the larger heater by 1
        if (new_total > USB_MAX_COMBINED_PWM) {
            if (data.sample_heater_pwm_value > data.valve_heater_pwm_value) {
                data.sample_heater_pwm_value--;
            } else {
                data.valve_heater_pwm_value--;
            }
        }
    }
#endif
}

/**
 * @brief Apply phase-shifted heater control to reduce peak power draw
 * 
 * When enabled, this function alternates the amplification and valve heaters
 * out of phase over a defined time period. This time-domain power distribution
 * reduces peak instantaneous current draw while maintaining average heating power.
 * 
 * Phase Control Strategy:
 * ----------------------
 * 1. Track elapsed time using phase counter
 * 2. Calculate current phase position within period (0.0 to 1.0)
 * 3. First half of period:  Amplification heater active, Valve heater off
 * 4. Second half of period: Valve heater active, Amplification heater off
 * 5. Heaters alternate continuously with 50% duty cycle each
 * 
 * Timing Configuration:
 * --------------------
 * - Phase Period:      HEATER_PHASE_PERIOD_MS (default 1000ms = 1 second)
 * - Phase Duty Cycle:  HEATER_PHASE_DUTY_PERCENT (default 50%)
 * - Update Rate:       PID_TIMER_INTERVAL (500ms)
 * 
 * Example Timeline (1-second period):
 * ----------------------------------
 * Time (ms) | Phase | Amp Heater | Valve Heater
 * ----------|-------|------------|-------------
 *    0-499  |  0%   |  ACTIVE    |  OFF
 *  500-999  | 50%   |  OFF       |  ACTIVE
 * 1000-1499 |  0%   |  ACTIVE    |  OFF (cycle repeats)
 * 
 * Power Management Benefits:
 * -------------------------
 * - Reduces peak instantaneous current draw by ~50%
 * - Maintains average heating power over time
 * - Complements USB power limiting (magnitude + time domain)
 * - Prevents both heaters from drawing max current simultaneously
 * - Smooths power supply load and reduces electrical noise
 * 
 * Temperature Control Considerations:
 * ----------------------------------
 * - Each heater receives heating for 50% of time
 * - Thermal mass of heaters smooths temperature variations
 * - PID controllers adjust output to maintain target temperatures
 * - May require increased proportional gain to compensate for duty cycle
 * - Average temperature control remains stable
 * 
 * Integration with USB Power Limiting:
 * -----------------------------------
 * - Phase shifting is applied AFTER USB power limiting
 * - USB limiting caps instantaneous total power (magnitude domain)
 * - Phase shifting spreads power over time (time domain)
 * - Combined effect: Both magnitude and time-domain power management
 * - USB limiting ensures compliance even if phase shift disabled
 * 
 * @note This function modifies data.sample_heater_pwm_value and data.valve_heater_pwm_value
 * @note Only active when data.heater_phase_shift_mode is true
 * @note Updates data.heater_phase_counter_ms to track phase position
 * @note Phase counter wraps at HEATER_PHASE_PERIOD_MS
 * @note Works on PWM values already processed by USB power limiting
 * 
 * @warning Temperature control loop must be tuned for 50% duty cycle operation
 * @warning Rapid temperature changes may be slower due to reduced duty cycle
 * @warning Not recommended for applications requiring fast thermal response
 */
static void Apply_Phase_Shift_Control(void)
{
#if HEATER_PHASE_SHIFT_ENABLED
    // Only apply phase shifting when mode is enabled
    if (!data.heater_phase_shift_mode) {
        return;
    }
    
    // Update phase counter (increments by PID_TIMER_INTERVAL each call)
    data.heater_phase_counter_ms += PID_TIMER_INTERVAL;
    
    // Wrap phase counter at period boundary
    if (data.heater_phase_counter_ms >= HEATER_PHASE_PERIOD_MS) {
        data.heater_phase_counter_ms = 0;
    }
    
    // Save original PWM values before phase gating
    uint8_t sample_pwm_original = data.sample_heater_pwm_value;
    uint8_t valve_pwm_original = data.valve_heater_pwm_value;
    
    // Calculate phase position (0-100% of period)
    // Using integer math: phase_percent = (counter * 100) / period
    uint8_t phase_percent = (data.heater_phase_counter_ms * 100) / HEATER_PHASE_PERIOD_MS;
    
    // Apply phase-shifted gating: Only ONE heater active at a time
    // IMPORTANT: Always zero out BOTH heaters first to prevent overlap
    data.sample_heater_pwm_value = 0;
    data.valve_heater_pwm_value = 0;
    
    // Determine which heater should be active based on phase position
    // Using strict inequality to ensure clean phase boundaries
    if (phase_percent < HEATER_PHASE_DUTY_PERCENT) {
        // First half of period (0-49%): Amplification heater active only
        data.sample_heater_pwm_value = sample_pwm_original;
        // Valve explicitly kept at 0
    } else {
        // Second half of period (50-100%): Valve heater active only  
        data.valve_heater_pwm_value = valve_pwm_original;
        // Amplification explicitly kept at 0
    }
    
    // Note: This ensures ONLY ONE heater is ever active at a time.
    // The active heater uses its PID-computed and USB-limited PWM value.
    // The inactive heater is forced to 0 to prevent simultaneous operation.
#endif
}

/**
 * @brief Updates the PID controllers for sample and valve heaters.
 */
void Update_PID(void)
{    
    pid_controller_compute(SAMPLE_HEATER, data.sample_temperature_c);
    pid_controller_compute(VALVE_HEATER, data.valve_temperature_c);

    data.sample_heater_pwm_value = 128;
    data.valve_heater_pwm_value = 128;
    
    // Get PID outputs directly
    #ifdef DEBUG_HEATERS
        data.sample_heater_pwm_value = 0;       // disable SH
        data.valve_heater_pwm_value = VH_FIXED_PWM_TEST;
    #else        
        data.sample_heater_pwm_value = pid_data[SAMPLE_HEATER].out;
        data.valve_heater_pwm_value = pid_data[VALVE_HEATER].out;
    #endif

    // Apply USB power limiting if enabled
    // This ensures total power draw stays within USB current specifications
    Apply_USB_Power_Limiting();

    // Apply phase-shifted heater control if enabled
    // This alternates heaters over time to reduce peak instantaneous power draw
    // Must be called AFTER USB power limiting to work on already-limited values
    Apply_Phase_Shift_Control();

    // Update PWM duty cycles for sample heater
    if (pwm_amp_ctrl.enabled) {
        if (pwm_amp_ctrl.heater_level_high) {
            // High-side control: PWM CH1 high, CH2 low
            PWM_Set_Sample_Heater_Channels(data.sample_heater_pwm_value, 0);
        } else {
            // Low-side control: PWM CH1 low, CH2 high
            PWM_Set_Sample_Heater_Channels(0, data.sample_heater_pwm_value);
        }
    } else {
        PWM_Set_Sample_Heater_Channels(0, 0);
    }

    // Update PWM duty cycles for valve heater
    if (pwm_valve_ctrl.enabled) {//&& data.valve_heater_pwm_value > 0
        if (pwm_valve_ctrl.heater_level_high) {
            // High-side control: PWM CH1 high, CH2 low
            PWM_Set_Valve_Heater_Channels(data.valve_heater_pwm_value, 0);
        } else {
            // Low-side control: PWM CH1 low, CH2 high
            PWM_Set_Valve_Heater_Channels(0, data.valve_heater_pwm_value);
        }
    } else {
        PWM_Set_Valve_Heater_Channels(0, 0);
    }

    /*
    // Update PWM duty cycles for sample heater
    if (pwm_amp_ctrl.enabled && data.sample_heater_pwm_value > 0) {
        if (pwm_amp_ctrl.heater_level_high) {
            // High-side control: PWM CH1 high, CH2 low
            PWM_Set_Sample_Heater_Channels(data.sample_heater_pwm_value, 0);
        } else {
            // Low-side control: PWM CH1 low, CH2 high
            PWM_Set_Sample_Heater_Channels(0, data.sample_heater_pwm_value);
        }
    } else {
        PWM_Set_Sample_Heater_Channels(0, 0);
    }
        

    
        */
}

void print_log_data(void) 
{
#if defined(BOARDCONFIG_MK5AA) || defined(BOARDCONFIG_MK6AA) || defined(BOARDCONFIG_MK6F)
    //sprintf(outputStr, "%4d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %d %d %d\r\n", data.msec_test_count, data.sample_temperature_c, pid_data[SAMPLE_HEATER].setpoint, data.sh_pwm_during_adc_meas, data.valve_temperature_c, pid_data[VALVE_HEATER].setpoint, data.vh_pwm_during_adc_meas, data.system_input_voltage, data.state, (int) pid_data[VALVE_HEATER].integrator,(int) pid_data[VALVE_HEATER].dTerm);
    sprintf(outputStr, "%4d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %d\r\n", data.msec_test_count, data.sample_temperature_c, pid_data[SAMPLE_HEATER].setpoint, data.sh_pwm_during_adc_meas, data.valve_temperature_c, pid_data[VALVE_HEATER].setpoint, data.vh_pwm_during_adc_meas, data.system_input_voltage, data.state);
#else
    //sprintf(outputStr, "%4d, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %d %d %d\r\n", data.msec_test_count, data.sample_temperature_c, pid_data[SAMPLE_HEATER].setpoint, data.sh_pwm_during_adc_meas, data.valve_temperature_c, pid_data[VALVE_HEATER].setpoint, data.vh_pwm_during_adc_meas, data.vcc_mcu_voltage, data.state, (int) pid_data[VALVE_HEATER].integrator,(int) pid_data[VALVE_HEATER].dTerm);
    sprintf(outputStr, "%4lu, %1.2f, %1.2f, %d, %1.2f, %1.2f, %d, %1.2f, %d\r\n", data.msec_test_count, data.sample_temperature_c, pid_data[SAMPLE_HEATER].setpoint, data.sh_pwm_during_adc_meas, data.valve_temperature_c, pid_data[VALVE_HEATER].setpoint, data.vh_pwm_during_adc_meas, data.vcc_mcu_voltage, data.state);
#endif
    
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);    
}

void send_max_temps(void) {
    sprintf(outputStr, "SH_MAX: %1.2f VH_MAX: %1.2f sample_ramp_time: %lu valve_ramp_time: %lu\r\n", data.sample_max_temperature_c, data.valve_max_temperature_c, data.sample_ramp_time_sec, data.valve_ramp_time_sec);
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


// Timer ISR functions are called from the HAL_TIM ISR.
// Only time critical code should be run in these routines.
// It is recommended to set a flag in the ISR that enables lower priority code to run in user space.

void PWMTimer_ISR(void)
{   
    // Hardware PWM update is handled by Update_PID now
}

void LEDTimer_ISR(void)
{
    GPIO_PinState ledState;

    // Default to LED on (PIN_RESET) unless in specific states
    if (data.state == self_test_1 || data.state == self_test_2 || data.state == preheat) {
        ledState = GPIO_PIN_SET;         // LED off during self-test and preheat
    } else if (data.state == amplification_ramp) {
        ledState = GPIO_PIN_RESET;       // LED on during amplification_ramp
    } else if (data.state == amplification) {
        ledState = GPIO_PIN_SET;         // LED off during amplification
    } else if (data.state == actuation || data.state == actuation_ramp) {
        ledState = GPIO_PIN_RESET;       // LED on during actuation
    } else if (data.state == detection) {
        ledState = GPIO_PIN_SET;         // LED off during detection
    } else if (data.minute_test_count == 0) {
        ledState = GPIO_PIN_RESET;       // LED on after system setup
    } else {
        // Toggle LED in other states
        ledState = (HAL_GPIO_ReadPin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1) == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, ledState);
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
    // Sample button state
    GPIO_PinState currentButtonState = HAL_GPIO_ReadPin(Pins.GPIOx_PUSHBUTTON, Pins.GPIO_Pin_PUSHBUTTON);
    
    // Detect falling edge (button press)
    if (currentButtonState == GPIO_PIN_RESET && last_pushbutton_value == GPIO_PIN_SET) {
        flags.flagPushbutton = true;
    }
    
    last_pushbutton_value = currentButtonState;
}

// USB CC1 and CC2 readings: Per the USB standard only one will be valid. (USB cables typically tie either CC1 or CC2 to ground)
// An error will be thrown if neither CC1 nor CC2 reads below 0.75v and 1.75v
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






/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
