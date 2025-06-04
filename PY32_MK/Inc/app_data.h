/*
*   File: app_data.h
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*/

#ifndef APP_DATA_H_
#define APP_DATA_H_

#include <stdbool.h>
#include "main.h"
#include "timers.h"

#define AMPLIFICATION_TIME_MIN          15
#define ACTUATION_PREP_TIME_MIN         1    // Option to change the controls during the last minute of amplification.
#define ACTUATION_TIME_MIN              5
#define DETECTION_TIME_MIN              1

#define SELFTEST_TIME_MSEC              12000
#define SELFTEST_MIN_TEMP_RISE_C        4.0
#define PREHEAT_TEMP_C    	            30	    // Start the heaters in non simultaneous mode until they hit this temperature
#define PREHEAT_MAX_TIME_MSEC           150000  // Go to a failure state if the board cannot preheat within this amount of time.

#define BOARDCONFIG_MK7R

#define FW_VERSION_STR                  "FW:v1.0"


#if defined(BOARDCONFIG_MK5AA) 
    #define BUILD_HW_STR                    "HW:MK5AA"
#elif defined(BOARDCONFIG_MK5C)
    #define BUILD_HW_STR                    "HW:MK5C"
#elif defined(BOARDCONFIG_MK6AA)
    #define BUILD_HW_STR                    "HW:MK6AA"
#elif defined(BOARDCONFIG_MK6C)
    #define BUILD_HW_STR                    "HW:MK6C"
#elif defined(BOARDCONFIG_MK6F)
    #define BUILD_HW_STR                    "HW:MK6F"
#elif defined(BOARDCONFIG_MK7R)
    #define BUILD_HW_STR                    "HW:MK7R_B001"
#elif defined(BOARDCONFIG_MK7C)
    #define BUILD_HW_STR                    "HW:MK7C"
#else
    FAIL -- invalid board
#endif


//#define PUSHBUTTON_UI_ENABLED           1      

#if defined(BOARDCONFIG_MK5AA) || defined(BOARDCONFIG_MK6AA) 
    #define VCC_MCU_MIN_VOLTAGE     2.2
    #define VCC_MCU_MAX_VOLTAGE     3.6
#elif defined(BOARDCONFIG_MK6F)
    #define VCC_MCU_MIN_VOLTAGE     4.5
    #define VCC_MCU_MAX_VOLTAGE     5.5
#elif defined(BOARDCONFIG_MK5C) || defined(BOARDCONFIG_MK6C)
    #define VCC_MCU_MIN_VOLTAGE     2.4
    #define VCC_MCU_MAX_VOLTAGE     2.6
#elif defined(BOARDCONFIG_MK7R) || defined(BOARDCONFIG_MK7C)
    #define VCC_MCU_MIN_VOLTAGE     2.4
    #define VCC_MCU_MAX_VOLTAGE     2.6
#else
    FAIL -- invalid board type
#endif

#if defined(BOARDCONFIG_MK5C) || defined(BOARDCONFIG_MK6C) || defined(BOARDCONFIG_MK5AA) || defined(BOARDCONFIG_MK6AA) || defined(BOARDCONFIG_MK6F)
#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   72
#define VALVE_ZONE_AMP_SOAK_TARGET_C    72
#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 0
#define VALVE_ZONE_VALVE_PREP_TARGET_C  72
#define VALVE_ZONE_VALVE_SOAK_TARGET_C  101
#define COLD_TEMP_SETPOINT_OFFSET_C     2           //cold temp values are dependent on the outer device packaging (convective shielding and insulation)
#define COLD_TEMP_OFFSET_THRESHOLD_C    14
#define AMPLIFICATION_MIN_VALID_TEMP_C  65
#define ACTUATION_MIN_VALID_TEMP_C      95
#define HEATER_SHUTDOWN_C               0
#define HEATER_ELEMENT_POWER_RATIO      35
#define OVERTEMP_ERR_C                  110
#define SLEW_RATE_LIMIT                 250

#elif defined(BOARDCONFIG_MK7R) || defined(BOARDCONFIG_MK7C)
#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   72
#define VALVE_ZONE_AMP_SOAK_TARGET_C    72
#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 0
#define VALVE_ZONE_VALVE_PREP_TARGET_C  72
#define VALVE_ZONE_VALVE_SOAK_TARGET_C  101
#define COLD_TEMP_SETPOINT_OFFSET_C     2           //cold temp values are dependent on the outer device packaging (convective shielding and insulation)
#define COLD_TEMP_OFFSET_THRESHOLD_C    14
#define AMPLIFICATION_MIN_VALID_TEMP_C  65
#define ACTUATION_MIN_VALID_TEMP_C      95
#define HEATER_SHUTDOWN_C               0
#define HEATER_ELEMENT_POWER_RATIO      50
#define OVERTEMP_ERR_C                  110
#define SLEW_RATE_LIMIT                 250
#else
    FAIL -- invalid board type
#endif
/*MK Generation Calibration CONST*/


// Timer related
#define PWM_TIMER_INTERVAL              1L
#define MINUTE_TIMER_INTERVAL           (60000L * TICKS_PER_MSEC)
#define LOG_TIMER_INTERVAL              (500L * TICKS_PER_MSEC)

#define PID_TIMER_INTERVAL              (500L * TICKS_PER_MSEC)
#define DATA_COLLECTION_TIMER_INTERVAL  (500L * TICKS_PER_MSEC)
#define LED_TIMER_INTERVAL_AMPLIFICATION (500L * TICKS_PER_MSEC)
#define LED_TIMER_INTERVAL_ACTIVATION   (250L * TICKS_PER_MSEC)
#define LED_TIMER_INTERVAL_DONE         (1000L * TICKS_PER_MSEC)
#define PUSHBUTTON_TIMER_INTERVAL       (10L * TICKS_PER_MSEC)
                                        
#define STARTUP_DELAY_MS                (5000L * TICKS_PER_MSEC)

#define PWM_MAX                         250

#define NUMPROCESS                      5  

// Data Structures
struct CONTROL 
{
    float setpoint;
    float input;
    float output;
    float kp;
    float ki;
    float kd;
};

typedef struct app_data_t {
    // structure containing application data, for passing through LOG and DEBUG interfaces
    bool test_active;
    bool heater_control_not_simultaneous;
    bool cold_ambient_temp_mode;
    uint8_t H1_pwm_value;
    uint8_t H2_pwm_value;
    uint8_t H3_pwm_value;
    uint8_t H4_pwm_value;
    float self_test_H1_start_temp_c;
    float self_test_H2_start_temp_c;
    float self_test_H3_start_temp_c;
    float self_test_H4_start_temp_c;
    float H1_thermistor_v;
    uint32_t H1_thermistor_r;
    float H1_temperature_c;
    float H2_thermistor_v;
    uint32_t H2_thermistor_r;
    float H2_temperature_c;
    float H3_thermistor_v;
    uint32_t H3_thermistor_r;
    float H3_temperature_c;
    float H4_thermistor_v;
    uint32_t H4_thermistor_r;
    float H4_temperature_c;
    float py32_temperature_c;
    uint8_t state;
    uint8_t alarm;
    volatile uint32_t msec_tick_count;
    volatile uint32_t msec_test_count;
    volatile uint32_t minute_test_count;

    uint32_t adcReading[9];
    float adcVoltage[9];         

    float vcc_mcu_voltage;       
    float system_input_voltage;
    float H1_max_temperature_c;
    float H2_max_temperature_c;
    float H3_max_temperature_c;
    float H4_max_temperature_c;
    bool usb_cc_adc_read_enabled;
    float usb_cc1_voltage;
    float usb_cc2_voltage;
    uint32_t valve_ramp_time;
    uint32_t test_interval;
	uint8_t H1_pwm_during_adc_meas;
	uint8_t H2_pwm_during_adc_meas;
	uint8_t H3_pwm_during_adc_meas;
	uint8_t H4_pwm_during_adc_meas;
} app_data_t;

typedef struct flags_t {
    volatile bool flag_1minute;
    volatile bool flagDataCollection;
    volatile bool flagUpdatePID;
    volatile bool flagUpdateLED;
    volatile bool flagSendLogData;
    volatile bool flagDelayedStart;
    volatile bool flagPushbutton;
} flags_t;


#endif /*APP_DATA_H_*/