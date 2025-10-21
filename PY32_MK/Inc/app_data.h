/*
*   File: app_data.h
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*
*   Description:
*   Core application data structures and configuration constants for the NAATOS system.
*   This file defines system-wide settings, timing parameters, board configurations,
*   temperature targets, and the main data structures for system state tracking.
*/

#ifndef APP_DATA_H_
#define APP_DATA_H_

#include <stdbool.h>
#include "main.h"
#include "timers.h"

#define BOARDCONFIG_MK8

#define FW_VERSION_STR                  "FW:v1.6"

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
#elif defined(BOARDCONFIG_MK8)
    #define BUILD_HW_STR                    "HW:MK8"    
#else
    FAIL -- invalid board
#endif


// CONFIGURATION of OPTION BYTES - NOTE: Only used for Development. For production, flash once and set to 0 so firmware does not reset
//#define SET_OB_ONCE                     0

//#define ENABLE_POWER_ON_TESTS                 // Enable the power on tests, which will run at startup
#define IGNORE_RAMP_TIME                        // Start the test timer after the heater has ramped to near the setpoint temperature

/**
 * @brief Test Phase Timing Constants
 * 
 * These constants define the duration of each major test phase in minutes.
 */
#define AMPLIFICATION_TIME_MIN          17      /*!< Duration of amplification phase in minutes */
#define ACTUATION_TIME_MIN              5       /*!< Duration of actuation phase in minutes */
#define DETECTION_TIME_MIN              7       /*!< Duration of detection phase in minutes */

#define PWM_MAX                         255     /*!< Maximum PWM bit count */

/**
 * @brief Self-Test and Preheat Parameters
 * 
 * Configuration constants for system self-test and preheat operations.
 */
#define SELFTEST_TIME_MSEC              12000   /*!< Maximum duration for self-test sequence */
#define SELFTEST_MIN_TEMP_RISE_C        4.0     /*!< Minimum temperature rise required during self-test */
#define PREHEAT_TEMP_C    	            30	    /*!< Initial temperature target before main heating phase */
#define PREHEAT_MAX_TIME_MSEC           150000  /*!< Maximum allowed time for preheat phase */


//#define PUSHBUTTON_UI_ENABLED           1      

#if defined(BOARDCONFIG_MK5AA) || defined(BOARDCONFIG_MK6AA) 
    #define VCC_MCU_MIN_VOLTAGE     1.8
    #define VCC_MCU_MAX_VOLTAGE     3.6
#elif defined(BOARDCONFIG_MK6F)
    #define VCC_MCU_MIN_VOLTAGE     4.5         // The flex board runs directly from 5v USB
    #define VCC_MCU_MAX_VOLTAGE     5.5
#elif defined(BOARDCONFIG_MK5C) || defined(BOARDCONFIG_MK6C) || defined(BOARDCONFIG_MK8)
    #define VCC_MCU_MIN_VOLTAGE     2.4
    #define VCC_MCU_MAX_VOLTAGE     2.6
#else
    FAIL -- invalid board type
#endif

#if defined(BOARDCONFIG_MK5C) || defined(BOARDCONFIG_MK6C) || defined(BOARDCONFIG_MK5AA) || defined(BOARDCONFIG_MK6AA) || defined(BOARDCONFIG_MK6F) || defined(BOARDCONFIG_MK8)
/**
 * @brief Temperature Control Constants
 * 
 * Target temperatures and control parameters for different test phases
 * and operational modes of the sample and valve heaters.
 */
/* Amplification Phase Targets */
#define SAMPLE_ZONE_AMP_RAMP_TARGET_C   81      /*!< Sample heater target during amplification ramp */
#define VALVE_ZONE_AMP_RAMP_TARGET_C    74      /*!< Valve heater target during amplification ramp */

#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   63      /*!< Sample heater target during amplification soak */
#define VALVE_ZONE_AMP_SOAK_TARGET_C    63      /*!< Valve heater target during amplification soak */

/* Valve Phase Targets */
#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 0       /*!< Sample heater target during valve soak phase */
#define VALVE_ZONE_VALVE_SOAK_TARGET_C  89      /*!< Valve heater target during valve soak phase */
#define VALVE_ZONE_ACT_RAMP_TARGET_C    91      /*!< Valve heater target during actuation ramp */

/* Temperature Compensation */
#define COLD_TEMP_SETPOINT_OFFSET_C     2       /*!< Temperature adjustment for cold ambient conditions */
#define COLD_TEMP_OFFSET_THRESHOLD_C    14      /*!< Threshold for activating cold temperature mode */

/* Safety and Validation Limits */
#define AMPLIFICATION_MIN_VALID_TEMP_C  65      /*!< Minimum valid temperature during amplification */
#define ACTUATION_MIN_VALID_TEMP_C      95      /*!< Minimum valid temperature during actuation */
#define HEATER_RAMP_SETPOINT_OFFSET     1       /*!< Offset applied during temperature ramping */
#define HEATER_SHUTDOWN_C               0       /*!< Temperature threshold for heater shutdown */
#define HEATER_ELEMENT_POWER_RATIO      45      /*!< Power scaling factor for heater control */
#define OVERTEMP_ERR_C                  120     /*!< Over-temperature error threshold */
#define SLEW_RATE_LIMIT                 250     /*!< Maximum temperature change rate limit */

/* USB Power Limiting */
#define USB_POWER_LIMIT_ENABLED         1       /*!< Enable USB power budget management */
#define USB_MAX_COMBINED_PWM            191     /*!< Maximum combined PWM (75% of 255) for USB power limiting */
#define USB_PWM_SAFETY_MARGIN           10      /*!< Safety margin below max combined PWM */

/* Phase-Shifted Heater Control */
#define HEATER_PHASE_SHIFT_ENABLED      0       /*!< Enable out-of-phase heater operation */
#define HEATER_PHASE_PERIOD_MS          1000    /*!< Period for phase alternation (1 second) */
#define HEATER_PHASE_DUTY_PERCENT       50      /*!< Duty cycle percentage for each heater phase */

#else
    FAIL -- invalid board type
#endif
/*MK Generation Calibration CONST*/


/**
 * @brief System Timer Intervals
 * 
 * Defines the various timer intervals used for system operations.
 * All intervals are specified in system ticks (TICKS_PER_MSEC).
 */

/* Core Timer Intervals */
#define PWM_TIMER_INTERVAL              1L                      /*!< PWM update interval */
#define MINUTE_TIMER_INTERVAL           (60000L * TICKS_PER_MSEC) /*!< One minute timer interval */
#define LOG_TIMER_INTERVAL              (500L * TICKS_PER_MSEC)   /*!< Data logging interval */

/* Control and Monitoring Intervals */
#define PID_TIMER_INTERVAL              (500L * TICKS_PER_MSEC)   /*!< PID control loop update interval */
#define DATA_COLLECTION_TIMER_INTERVAL  (500L * TICKS_PER_MSEC)   /*!< Sensor data collection interval */

/* UI Update Intervals */
#define LED_TIMER_INTERVAL_AMPLIFICATION (500L * TICKS_PER_MSEC)  /*!< LED blink rate during amplification */
#define LED_TIMER_INTERVAL_ACTIVATION   (250L * TICKS_PER_MSEC)   /*!< LED blink rate during activation */
#define LED_TIMER_INTERVAL_DONE         (1000L * TICKS_PER_MSEC)  /*!< LED blink rate when test complete */
#define PUSHBUTTON_TIMER_INTERVAL       (10L * TICKS_PER_MSEC)    /*!< Pushbutton debounce interval */

/* System Operation Delays */                                        
#define STARTUP_DELAY_MS                (5000L * TICKS_PER_MSEC)   /*!< System initialization delay */
#define ACTUATION_DELAY_TIMER_INTERVAL  (20000L * TICKS_PER_MSEC)  /*!< Delay before actuation phase */


// Data Structures
/**
 * @brief PID Control parameters structure for heater control.
 * 
 * This structure contains all the parameters needed for temperature control
 * of a single heater zone, including PID gains, setpoints, and cold temperature
 * compensation settings.
 */
struct CONTROL 
{
    float setpoint;          /*!< Target temperature setpoint in degrees Celsius */
    float input;            /*!< Current measured temperature in degrees Celsius */
    float output;           /*!< Output value to the heater in PWM units (0-255) */
    float kp;              /*!< Proportional gain coefficient */
    float ki;              /*!< Integral gain coefficient */
    float kd;              /*!< Derivative gain coefficient */
    bool cold_temp_adjusted; /*!< Enable flag for cold ambient temperature compensation */
    float cold_temp_offset_c;/*!< Temperature offset when in cold ambient mode */
};

/**
 * @brief Main application data structure.
 * 
 * Contains all runtime data for the NAATOS system, including:
 * - System state and control flags
 * - Temperature measurements and calculations
 * - Timer counts and test progress
 * - ADC readings and derived values
 * - Power supply and USB status
 * - Test statistics and timing data
 * 
 * This structure is used for both operational control and diagnostic logging.
 */
typedef struct app_data_t {
    /* System State */
    bool test_active;                    /*!< True when test is running */
    bool cold_ambient_temp_mode;          /*!< True when operating in cold temperature mode */
    bool usb_power_limit_mode;            /*!< True when USB power budget limiting is active */
    bool heater_phase_shift_mode;         /*!< True when phase-shifted heater operation is active */
    
    /* Heater Control */
    uint8_t sample_heater_pwm_value;      /*!< Current PWM duty cycle for sample heater */
    uint8_t valve_heater_pwm_value;       /*!< Current PWM duty cycle for valve heater */
    uint32_t heater_phase_counter_ms;     /*!< Phase counter for alternating heater operation */
    
    /* Temperature Data */
    float self_test_sh_start_temp_c;      /*!< Initial sample heater temperature for self-test */
    float self_test_vh_start_temp_c;      /*!< Initial valve heater temperature for self-test */
    float sample_thermistor_v;            /*!< Sample thermistor voltage reading */
    uint32_t sample_thermistor_r;         /*!< Calculated sample thermistor resistance */
    float sample_temperature_c;           /*!< Calculated sample temperature in Celsius */
    float valve_thermistor_v;             /*!< Valve thermistor voltage reading */
    uint32_t valve_thermistor_r;          /*!< Calculated valve thermistor resistance */
    float valve_temperature_c;            /*!< Calculated valve temperature in Celsius */
    float py32_temperature_c;             /*!< MCU internal temperature in Celsius */
    
    /* System Status */
    uint8_t state;                        /*!< Current state machine state */
    uint8_t alarm;                        /*!< Current alarm status if any */
    
    /* Timing Counters */
    volatile uint32_t msec_tick_count;    /*!< System uptime in milliseconds */
    volatile uint32_t msec_test_count;    /*!< Current test duration in milliseconds */
    volatile uint32_t minute_test_count;  /*!< Current test duration in minutes */

    /* ADC Measurements */
    uint32_t adcReading[7];            /*!< Raw ADC readings for multiple channels */
    float adcVoltage[7];               /*!< Converted voltage values for ADC channels */

    /* Power Supply and System Status */
    bool system_on_usb_power;          /*!< True if system is powered via USB */
    float vcc_mcu_voltage;             /*!< Measured MCU supply voltage */
    float system_input_voltage;        /*!< Main system input voltage */
    float sample_max_temperature_c;    /*!< Peak temperature recorded for sample heater */
    float valve_max_temperature_c;     /*!< Peak temperature recorded for valve heater */
    
    /* USB Configuration Channel Status */
    bool usb_cc_adc_read_enabled;      /*!< True if USB CC pin monitoring is active */
    float usb_cc1_voltage;             /*!< USB Configuration Channel 1 voltage */
    float usb_cc2_voltage;             /*!< USB Configuration Channel 2 voltage */
    
    /* Timing and Progress Tracking */
    uint32_t sample_ramp_start_time_msec; /*!< Timestamp when sample heater started ramping */
    uint32_t sample_ramp_time_sec;     /*!< Duration of sample heater ramp phase */
    uint32_t valve_ramp_start_time_msec;/*!< Timestamp when valve heater started ramping */
    uint32_t valve_ramp_time_sec;      /*!< Duration of valve heater ramp phase */
    uint32_t test_interval;            /*!< Current test phase interval */
    
    /* Heater PWM Status During ADC */
    uint8_t sh_pwm_during_adc_meas;    /*!< Sample heater PWM value during ADC measurement */
    uint8_t vh_pwm_during_adc_meas;    /*!< Valve heater PWM value during ADC measurement */

    /* Ramp Status Flags */
    bool flag_reached_actuation_ramp_target; /*!< True when actuation temperature is reached */
} app_data_t;

/**
 * @brief System timing and event flags structure.
 * 
 * Contains volatile boolean flags used to coordinate timing-based events
 * and system operations. These flags are typically set by timer interrupts
 * and cleared by the main processing loop.
 */
typedef struct flags_t {
    volatile bool flag_1minute;         /*!< Set every minute for periodic tasks */
    volatile bool flagDataCollection;   /*!< Triggers data sampling and logging */
    volatile bool flagUpdatePID;        /*!< Signals PID control loop update needed */
    volatile bool flagUpdateLED;        /*!< Indicates LED status needs updating */
    volatile bool flagSendLogData;      /*!< Triggers sending of logged data */
    volatile bool flagDelayedStart;     /*!< Controls delayed startup sequence */
    volatile bool flagPushbutton;       /*!< Set when pushbutton state changes */
    volatile bool flagActuationDelay;   /*!< Controls actuation timing delays */
} flags_t;


#endif /*APP_DATA_H_*/