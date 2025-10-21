/**
 ******************************************************************************
 * @file    pwm_init.h
 * @author  Global Health Labs
 * @brief   Header file for PWM heater control module
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * This file provides the public API for PWM-based heater control in the
 * NAATOS system. The implementation uses hardware timers on the PY32F003x8
 * microcontroller with special handling for hardware constraints.
 *
 * Key Features:
 * - Independent control of amplification heaters (PB6/PB7)
 * - Multiplexed control of valve heaters (PF0/PF1)
 * - 8-bit PWM resolution (0-255 duty cycle)
 * - ~9.375 kHz PWM frequency
 *
 * Hardware Architecture:
 * ----------------------
 * Amplification Heaters (Independent):
 *   PB6 (AMP_CTRL1): TIM16_CH1N via AF2
 *   PB7 (AMP_CTRL2): TIM17_CH1N via AF2
 *   → Both can operate simultaneously with different duty cycles
 *
 * Valve Heaters (Multiplexed):
 *   PF0 (VALVE_CTRL1): TIM14_CH1 via AF2
 *   PF1 (VALVE_CTRL2): TIM14_CH1 via AF13
 *   → Only ONE can be active at a time (hardware constraint)
 *
 * Usage Example:
 * --------------
 * @code
 * #include "pwm_init.h"
 * 
 * // Initialize PWM system
 * PWM_Init();
 * 
 * // Control amplification heaters (can be independent)
 * PWM_Set_Sample_Heater_Channels(200, 150);  // CH1=78%, CH2=59%
 * 
 * // Control valve heaters (mutually exclusive)
 * PWM_Set_Valve_Heater_Channels(180, 0);     // Valve 1 at 71%
 * PWM_Set_Valve_Heater_Channels(0, 200);     // Switch to Valve 2 at 78%
 * 
 * // Turn off all heaters
 * PWM_Set_Sample_Heater_Channels(0, 0);
 * PWM_Set_Valve_Heater_Channels(0, 0);
 * @endcode
 *
 * @see pwm_init.c for implementation details
 ******************************************************************************
 */

#ifndef PWM_INIT_H_
#define PWM_INIT_H_

//#define PY32F003x8

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "py32f0xx.h"
#include "py32f003x8.h"

/* Exported timer handles ----------------------------------------------------*/
/**
 * @brief Timer handle for TIM14 (Valve heater PWM control)
 * 
 * Used for multiplexed PWM output to PF0/PF1. Only one valve heater
 * can be active at a time due to hardware sharing of TIM14_CH1.
 */
extern TIM_HandleTypeDef htim14;

/**
 * @brief Timer handle for TIM16 (Amplification heater 1 PWM control)
 * 
 * Generates PWM on PB6 using complementary output CH1N. Independent
 * control - can operate simultaneously with TIM17.
 */
extern TIM_HandleTypeDef htim16;

/**
 * @brief Timer handle for TIM17 (Amplification heater 2 PWM control)
 * 
 * Generates PWM on PB7 using complementary output CH1N. Independent
 * control - can operate simultaneously with TIM16.
 */
extern TIM_HandleTypeDef htim17;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize PWM hardware for heater control
 * 
 * Configures three hardware timers for PWM generation:
 * - TIM14: Valve heater control with GPIO multiplexing (PF0/PF1)
 * - TIM16: Amplification heater 1 using complementary output (PB6)
 * - TIM17: Amplification heater 2 using complementary output (PB7)
 * 
 * All timers configured for:
 * - PWM frequency: ~9.375 kHz (24MHz / (10 * 256))
 * - Resolution: 8-bit (0-255 duty cycle)
 * - Initial state: All heaters off (0% duty cycle)
 * 
 * @note Must be called before using PWM_Set_*_Heater_Channels() functions
 * @note Calls APP_ErrorHandler() if timer initialization fails
 * @note GPIO clocks and timer clocks are enabled automatically
 * 
 * @see PWM_Set_Sample_Heater_Channels()
 * @see PWM_Set_Valve_Heater_Channels()
 */
void PWM_Init(void);

/**
 * @brief Set PWM duty cycle for amplification heater channels
 * 
 * Controls both amplification heater elements independently using
 * TIM16 (PB6) and TIM17 (PB7). Both heaters can be active simultaneously.
 * 
 * @param ch1_duty PWM duty cycle for amplification heater 1 (0-255)
 *                 - Range: 0 (off) to 255 (100% duty cycle)
 *                 - Maps to PB6/AMP_CTRL1 via TIM16_CH1N
 * 
 * @param ch2_duty PWM duty cycle for amplification heater 2 (0-255)
 *                 - Range: 0 (off) to 255 (100% duty cycle)
 *                 - Maps to PB7/AMP_CTRL2 via TIM17_CH1N
 * 
 * @note Both channels are independent - can have different duty cycles
 * @note Changes take effect immediately (next PWM period ~107 µs)
 * @note Safe to call at high frequency (no rate limiting needed)
 * @note PWM_Init() must be called first
 * 
 * Usage Example:
 * @code
 * PWM_Set_Sample_Heater_Channels(200, 150);  // CH1=78%, CH2=59%
 * PWM_Set_Sample_Heater_Channels(0, 0);      // Both off
 * @endcode
 */
void PWM_Set_Sample_Heater_Channels(uint8_t ch1_duty, uint8_t ch2_duty);

/**
 * @brief Set PWM duty cycle for valve heater channels
 * 
 * Controls valve heater elements using GPIO multiplexing. Due to hardware
 * constraints, only ONE valve heater can be active at any time. The function
 * automatically handles GPIO alternate function switching.
 * 
 * Multiplexing Behavior:
 * - If ch1 > 0 and ch2 = 0: Enable PF0 (VALVE_CTRL1) only
 * - If ch2 > 0 and ch1 = 0: Enable PF1 (VALVE_CTRL2) only
 * - If both > 0:            Priority to CH1 (PF0), CH2 ignored
 * - If both = 0:            Both disabled
 * 
 * @param ch1_duty PWM duty cycle for valve heater 1 (0-255)
 *                 - Range: 0 (off) to 255 (100% duty cycle)
 *                 - Maps to PF0/VALVE_CTRL1 via TIM14_CH1/AF2
 * 
 * @param ch2_duty PWM duty cycle for valve heater 2 (0-255)
 *                 - Range: 0 (off) to 255 (100% duty cycle)
 *                 - Maps to PF1/VALVE_CTRL2 via TIM14_CH1/AF13
 * 
 * @note Only ONE valve heater can be active at a time (hardware limitation)
 * @note Application must ensure mutual exclusion (not enforced by function)
 * @note GPIO switching has microsecond latency (negligible for control loops)
 * @note PWM_Init() must be called first
 * 
 * @warning Do not request both heaters simultaneously - CH1 takes priority
 * @warning Application should implement mutual exclusion logic
 * 
 * Usage Example:
 * @code
 * PWM_Set_Valve_Heater_Channels(180, 0);   // Valve 1 active at 71%
 * PWM_Set_Valve_Heater_Channels(0, 200);   // Switch to Valve 2 at 78%
 * PWM_Set_Valve_Heater_Channels(0, 0);     // Both off
 * @endcode
 */
void PWM_Set_Valve_Heater_Channels(uint8_t ch1_duty, uint8_t ch2_duty);

#ifdef __cplusplus
}
#endif

#endif /* PWM_INIT_H_ */