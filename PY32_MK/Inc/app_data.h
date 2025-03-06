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
#define ACTUATION_PREP_TIME_MIN         1    // Pre-heat VH during the last minute of amplification.
#define ACTUATION_TIME_MIN              5
#define DETECTION_TIME_MIN              1

#define SELFTEST_TIME_MSEC              10000
#define SELFTEST_MIN_TEMP_RISE_C        5
#define PREHEAT_TEMP_C    	            40	//  Start the heaters in non simultaneous mode until they hit this temperature

#define BOARDCONFIG_MK6F

#define FW_VERSION_STR                  "FW:v0.5"

//#define PUSHBUTTON_UI_ENABLED           1      

/*MK Generation Calibration CONST*/
#ifdef BOARDCONFIG_MK1_1
#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   68
#define VALVE_ZONE_AMP_SOAK_TARGET_C    69
#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 68
#define VALVE_ZONE_VALVE_PREP_TARGET_C  70
#define VALVE_ZONE_VALVE_SOAK_TARGET_C  97
#define VALVE_ZONE_MIN_VALID_TEMP_C     89
#define HEATER_SHUTDOWN_C               0
#define OVERTEMP_ERR_C                  105
#define SLEW_RATE_LIMIT                 255
#define BUILD_HW_STR                    "HW:MK1_1_B10"

#elif defined(BOARDCONFIG_MK2)
#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   68
#define VALVE_ZONE_AMP_SOAK_TARGET_C    70
#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 68
#define VALVE_ZONE_VALVE_PREP_TARGET_C  70
#define VALVE_ZONE_VALVE_SOAK_TARGET_C  97
#define VALVE_ZONE_MIN_VALID_TEMP_C     89
#define HEATER_SHUTDOWN_C               0
#define OVERTEMP_ERR_C                  105
#define SLEW_RATE_LIMIT                 255
#define BUILD_HW_STR                    "HW:MK2_Bx"

#elif defined(BOARDCONFIG_MK3)
#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   68
#define VALVE_ZONE_AMP_SOAK_TARGET_C    50
#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 40
#define VALVE_ZONE_VALVE_PREP_TARGET_C  70
#define VALVE_ZONE_VALVE_SOAK_TARGET_C  97
#define VALVE_ZONE_MIN_VALID_TEMP_C     89
#define HEATER_SHUTDOWN_C               0
#define OVERTEMP_ERR_C                  105
#define SLEW_RATE_LIMIT                 255
#define BUILD_HW_STR                    "HW:MK3_B4"

#elif defined(BOARDCONFIG_PY32_MK0)
#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   55
#define VALVE_ZONE_AMP_SOAK_TARGET_C    50
#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 40
#define VALVE_ZONE_VALVE_PREP_TARGET_C  70
#define VALVE_ZONE_VALVE_SOAK_TARGET_C  70
#define VALVE_ZONE_MIN_VALID_TEMP_C     65
#define HEATER_SHUTDOWN_C               0
#define OVERTEMP_ERR_C                  105
#define SLEW_RATE_LIMIT                 255
#define BUILD_HW_STR                    "HW:PY32_MK0_A"

#elif defined(BOARDCONFIG_MK4)
#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   68
#define VALVE_ZONE_AMP_SOAK_TARGET_C    70
#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 68
#define VALVE_ZONE_VALVE_PREP_TARGET_C  70
#define VALVE_ZONE_VALVE_SOAK_TARGET_C  97
#define VALVE_ZONE_MIN_VALID_TEMP_C     85
#define HEATER_SHUTDOWN_C               0
#define OVERTEMP_ERR_C                  105
#define SLEW_RATE_LIMIT                 255
#define BUILD_HW_STR                    "HW:MK4_Bx"

#elif defined(BOARDCONFIG_MK5C) || defined(BOARDCONFIG_MK6C) || defined(BOARDCONFIG_MK5AA) || defined(BOARDCONFIG_MK6F)
#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   76
#define VALVE_ZONE_AMP_SOAK_TARGET_C    76
#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 0
#define VALVE_ZONE_VALVE_PREP_TARGET_C  76
#define VALVE_ZONE_VALVE_SOAK_TARGET_C  105
//#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   68
//#define VALVE_ZONE_AMP_SOAK_TARGET_C    68
//#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 0
//#define VALVE_ZONE_VALVE_PREP_TARGET_C  68
//#define VALVE_ZONE_VALVE_SOAK_TARGET_C  97
#define VALVE_ZONE_MIN_VALID_TEMP_C     89
#define HEATER_SHUTDOWN_C               0
#define HEATER_ELEMENT_POWER_RATIO      35
#define OVERTEMP_ERR_C                  105
#define SLEW_RATE_LIMIT                 255
#define BUILD_HW_STR                    "HW:MK6F_B3"

#else

// DEFAULT-should work with all other env builds
#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   65
#define VALVE_ZONE_AMP_SOAK_TARGET_C    65
#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 65
#define VALVE_ZONE_VALVE_SOAK_TARGET_C  90
#define HEATER_SHUTDOWN_C               0
#define OVERTEMP_ERR_C                  105
#define SLEW_RATE_LIMIT                 255
#define BUILD_HW_STR                    "HW:OTHER"
#endif
/*MK Generation Calibration CONST*/


// Timer related
#define PWM_TIMER_INTERVAL              1L
#define MINUTE_TIMER_INTERVAL           (60000L * TICKS_PER_MSEC)
#define LOG_TIMER_INTERVAL              (1000L * TICKS_PER_MSEC)

#define PID_TIMER_INTERVAL              (500L * TICKS_PER_MSEC)
#define DATA_COLLECTION_TIMER_INTERVAL  (500L * TICKS_PER_MSEC)
#define LED_TIMER_INTERVAL_AMPLIFICATION (500L * TICKS_PER_MSEC)
#define LED_TIMER_INTERVAL_ACTIVATION   (250L * TICKS_PER_MSEC)
#define LED_TIMER_INTERVAL_DONE         (1000L * TICKS_PER_MSEC)
#define PUSHBUTTON_TIMER_INTERVAL       (10L * TICKS_PER_MSEC)
                                        
#define STARTUP_DELAY_MS                (5000L * TICKS_PER_MSEC)

#define PWM_MAX                         255

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
    uint8_t sample_heater_pwm_value;
    uint8_t valve_heater_pwm_value;
    float self_test_sh_start_temp_c;
    float self_test_vh_start_temp_c;
    float sample_thermistor_v;
    uint32_t sample_thermistor_r;
    float sample_temperature_c;
    float valve_thermistor_v;
    uint32_t valve_thermistor_r;
    float valve_temperature_c;
    float py32_temperature_c;
    uint8_t state;
    uint8_t alarm;
    volatile uint32_t msec_tick_count;
    volatile uint32_t msec_test_count;
    volatile uint32_t minute_test_count;

    uint32_t adcReading[7];
    float adcVoltage[7];         

    float vcc_mcu_voltage;       
    float system_input_voltage;
    float valve_max_temperature_c;
    bool usb_cc_adc_read_enabled;
    float usb_cc1_voltage;
    float usb_cc2_voltage;
    uint32_t valve_ramp_time;
    uint32_t test_interval;
	uint8_t sh_pwm_during_adc_meas;
	uint8_t vh_pwm_during_adc_meas;
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