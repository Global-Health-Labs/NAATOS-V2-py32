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

#define AMPLIFICATION_TIME_MIN          2
#define ACUTATION_PREP_TIME_MIN         1    // Pre-heat VH during the last minute of amplification.
#define ACUTATION_TIME_MIN              2
#define DETECTION_TIME_MIN              1

#define BOARDCONFIG_PY32_MK0
#define FW_VERSION_STR                  "FW:v0.1"

/*MK Generation Calibration CONST*/
#ifdef BOARDCONFIG_MK1_1
#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   68
#define VALVE_ZONE_AMP_SOAK_TARGET_C    69
#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 68
#define VALVE_ZONE_VALVE_PREP_TARGET_C  70
#define VALVE_ZONE_VALVE_SOAK_TARGET_C  97
#define VALVE_ZONE_MIN_VALID_TEMP_C     85
#define HEATER_SHUTDOWN_C               0
#define SLEW_RATE_LIMIT                 255
#define BUILD_HW_STR                    "HW:MK1_1_B10"
#elif defined(BOARDCONFIG_MK2)
#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   68
#define VALVE_ZONE_AMP_SOAK_TARGET_C    70
#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 68
#define VALVE_ZONE_VALVE_PREP_TARGET_C  70
#define VALVE_ZONE_VALVE_SOAK_TARGET_C  97
#define VALVE_ZONE_MIN_VALID_TEMP_C     85
#define HEATER_SHUTDOWN_C               0
#define SLEW_RATE_LIMIT                 255
#define BUILD_HW_STR                    "HW:MK2_Bx"
#elif defined(BOARDCONFIG_MK3)

#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   68
#define VALVE_ZONE_AMP_SOAK_TARGET_C    50
#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 40
#define VALVE_ZONE_VALVE_PREP_TARGET_C  70
#define VALVE_ZONE_VALVE_SOAK_TARGET_C  97
#define VALVE_ZONE_MIN_VALID_TEMP_C     85
#define HEATER_SHUTDOWN_C               0
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
#define SLEW_RATE_LIMIT                 255
#define BUILD_HW_STR                    "HW:MK4_Bx"

#else

// DEFAULT-should work with all other env builds
#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   65
#define VALVE_ZONE_AMP_SOAK_TARGET_C    65
#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 65
#define VALVE_ZONE_VALVE_SOAK_TARGET_C  90
#define HEATER_SHUTDOWN_C               0
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
#define LED_TIMER_INTERVAL              (200L * TICKS_PER_MSEC)
#define PUSHBUTTON_TIMER_INTERVAL       (10L * TICKS_PER_MSEC)
                                        
#define STARTUP_DELAY_MS                (5000L * TICKS_PER_MSEC)

#define PWM_MAX                         255

#define numProcess                      5  

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
    uint8_t sample_heater_pwm_value;
    uint8_t valve_heater_pwm_value;
    float sample_temperature_c;
    float valve_temperature_c;
    float py32_temperature_c;
    float test_adc_voltage;
    uint8_t state;
    uint8_t alarm;
    volatile uint32_t msec_tick_count;
    volatile uint32_t msec_test_count;
    volatile uint32_t minute_test_count;

    float battery_voltage;
    float valve_max_temperature_c;
    uint32_t valve_ramp_time;
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