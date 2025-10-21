/**
 ******************************************************************************
 * @file    pwm_init.c
 * @author  Global Health Labs
 * @brief   PWM initialization and control for NAATOS heater management
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * This module implements PWM control for the NAATOS heating system using
 * hardware timers on the PY32F003x8 microcontroller. The implementation
 * addresses hardware constraints where certain GPIO pins share timer channels,
 * requiring GPIO multiplexing and complementary output strategies.
 *
 * Hardware Architecture:
 * ----------------------
 * Timer Allocation:
 * - TIM14: Valve heater control (PF0/PF1) - Multiplexed single channel
 * - TIM16: Amplification heater 1 (PB6) - Complementary output CH1N
 * - TIM17: Amplification heater 2 (PB7) - Complementary output CH1N
 * - TIM3:  System timing (1ms ticks) - See timers.cpp
 *
 * GPIO Pin Mapping:
 * - PF0 (VALVE_CTRL1):  TIM14_CH1 via AF2  - Multiplexed
 * - PF1 (VALVE_CTRL2):  TIM14_CH1 via AF13 - Multiplexed
 * - PB6 (AMP_CTRL1):    TIM16_CH1N via AF2 - Independent (complementary output)
 * - PB7 (AMP_CTRL2):    TIM17_CH1N via AF2 - Independent (complementary output)
 *
 * Hardware Constraints:
 * ---------------------
 * 1. PF0 and PF1 can ONLY access TIM14_CH1 (not independent channels)
 *    - Solution: GPIO multiplexing - dynamically switch AF configuration
 *    - Application constraint: Only one valve heater active at a time
 *
 * 2. PB6 and PB7 use complementary outputs (CH1N) from TIM16/TIM17
 *    - CH1N outputs are typically for motor control dead-time insertion
 *    - Can be used as independent PWM outputs for this application
 *    - Both amplification heaters can operate simultaneously
 *
 * PWM Specifications:
 * -------------------
 * - Frequency: ~9.375 kHz (24MHz / (10 * 256))
 * - Resolution: 8-bit (0-255 duty cycle)
 * - Mode: PWM Mode 1 (active high)
 * - System Clock: 24 MHz
 *
 * Usage Example:
 * --------------
 * @code
 * PWM_Init();  // Initialize all PWM channels
 * 
 * // Set amplification heaters (can be independent)
 * PWM_Set_Sample_Heater_Channels(200, 150);  // Both can be active
 * 
 * // Set valve heaters (mutually exclusive)
 * PWM_Set_Valve_Heater_Channels(180, 0);     // Only CH1 active
 * PWM_Set_Valve_Heater_Channels(0, 200);     // Switch to CH2
 * @endcode
 *
 ******************************************************************************
 */

//#define PY32F003x8

#include "py32f0xx.h"
#include "py32f0xx_hal.h"
#include "py32f0xx_hal_tim.h"
#include "py32f0xx_hal_gpio.h"
#include "py32f0xx_hal_rcc.h"
#include "py32f003x8.h"
#include "py32f0xx_hal_cortex.h"
#include "pwm_init.h"
#include "gpio_init.h"
#include "error_handler.h"

/* Private variables ---------------------------------------------------------*/
/** @brief Timer handle for TIM14 - Valve heater PWM control (multiplexed) */
TIM_HandleTypeDef htim14;

/** @brief Timer handle for TIM16 - Amplification heater 1 PWM control */
TIM_HandleTypeDef htim16;

/** @brief Timer handle for TIM17 - Amplification heater 2 PWM control */
TIM_HandleTypeDef htim17;

/**
 * @brief Tracks which valve heater is currently active
 * 
 * Since PF0 and PF1 share TIM14_CH1, only one can output PWM at a time.
 * This variable tracks the current GPIO multiplexing state:
 * - 0: Both valves disabled (GPIO output low)
 * - 1: PF0 (VALVE_CTRL1) active, PF1 disabled
 * - 2: PF1 (VALVE_CTRL2) active, PF0 disabled
 */
static uint8_t active_valve = 0;

/* Public Functions ----------------------------------------------------------*/

/**
 * @brief Initialize PWM hardware for heater control
 * 
 * Configures three hardware timers for PWM generation:
 * - TIM14: Valve heater control with GPIO multiplexing (PF0/PF1)
 * - TIM16: Amplification heater 1 using complementary output (PB6)
 * - TIM17: Amplification heater 2 using complementary output (PB7)
 * 
 * All timers are configured for:
 * - PWM frequency: ~9.375 kHz
 * - Resolution: 8-bit (0-255 duty cycle)
 * - Initial duty cycle: 0% (heaters off)
 * 
 * Hardware Configuration Details:
 * -------------------------------
 * Timer Settings:
 * - Prescaler: 9 (divide 24MHz by 10 → 2.4MHz)
 * - Period: 255 (8-bit resolution)
 * - PWM Mode 1: Output high when CNT < CCR
 * 
 * GPIO Configuration:
 * - PB6/PB7: Configured immediately as alternate function outputs
 * - PF0/PF1: Configured dynamically based on which valve is active
 * 
 * Special Considerations:
 * ----------------------
 * 1. TIM16/TIM17 use complementary outputs (CH1N):
 *    - These are inverted outputs typically for motor control
 *    - OCNPolarity set to HIGH to match normal PWM behavior
 *    - Must use HAL_TIMEx_PWMN_Start() instead of HAL_TIM_PWM_Start()
 * 
 * 2. TIM14 output is multiplexed between PF0 and PF1:
 *    - Both pins cannot output PWM simultaneously
 *    - Application must ensure only one valve heater active at a time
 *    - See PWM_Set_Valve_Heater_Channels() for multiplexing logic
 * 
 * @note This function will call APP_ErrorHandler() if timer initialization fails
 * @note Valve GPIO pins (PF0/PF1) remain as GPIO outputs until first PWM command
 * 
 * @see PWM_Set_Sample_Heater_Channels()
 * @see PWM_Set_Valve_Heater_Channels()
 */
void PWM_Init(void)
{
    //TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    //TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable peripheral clocks
    __HAL_RCC_TIM14_CLK_ENABLE();
    __HAL_RCC_TIM16_CLK_ENABLE();
    __HAL_RCC_TIM17_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    // Configure timer base settings for 10kHz PWM frequency with 8-bit resolution
    // Timer clock = SystemCoreClock = 24MHz
    // PWM Freq = Timer clock / ((Prescaler + 1) * (Period + 1))
    // 10kHz = 24MHz / ((9 + 1) * (255 + 1)) = 9.375kHz (close enough)
    htim14.Instance = TIM14;
    htim14.Init.Prescaler = 9;
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Period = 255;
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
    {
        APP_ErrorHandler(ERR_FIRMWARE_CONFIG);
    }

    // Configure TIM16 for AMP_CTRL1 on PB6 (using CH1N complementary output)
    htim16.Instance = TIM16;
    htim16.Init.Prescaler = 9;
    htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim16.Init.Period = 255;
    htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim16.Init.RepetitionCounter = 0;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
    {
        APP_ErrorHandler(ERR_FIRMWARE_CONFIG);
    }

    // Configure TIM17 for AMP_CTRL2 on PB7 (using CH1N complementary output)
    htim17.Instance = TIM17;
    htim17.Init.Prescaler = 9;
    htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim17.Init.Period = 255;
    htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim17.Init.RepetitionCounter = 0;
    htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
    {
        APP_ErrorHandler(ERR_FIRMWARE_CONFIG);
    }

    // Configure PWM channel settings
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;  // Complementary output polarity
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    // Configure TIM14 channel for valve control (will be multiplexed between PF0 and PF1)
    if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        APP_ErrorHandler(ERR_FIRMWARE_CONFIG);
    }

    // Configure TIM16 channel for AMP_CTRL1 on PB6 (using CH1N complementary output)
    if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        APP_ErrorHandler(ERR_FIRMWARE_CONFIG);
    }

    // Configure TIM17 channel for AMP_CTRL2 on PB7 (using CH1N complementary output)
    if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        APP_ErrorHandler(ERR_FIRMWARE_CONFIG);
    }

    // Configure GPIO pins for PWM output
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // PF0 uses TIM14_CH1 via AF2 (valve multiplexing)
    // PF1 uses TIM14_CH1 via AF13 (valve multiplexing)
    // These will be configured dynamically when needed
    
    // PB6 uses TIM16_CH1N via AF2 (complementary output)
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM16;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // PB7 uses TIM17_CH1N via AF2 (complementary output)
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM17;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Start PWM generation (valve uses normal channel, amps use complementary channels)
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);  // Complementary output
    HAL_TIMEx_PWMN_Start(&htim17, TIM_CHANNEL_1);  // Complementary output
    
    // Set initial duty cycle to 0
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);
    
    // Initialize with no valve heaters active (amps are now independent)
    active_valve = 0;
}

/* Private Functions ---------------------------------------------------------*/

/**
 * @brief Switch TIM14_CH1 output to PF0 (VALVE_CTRL1)
 * 
 * Dynamically reconfigures GPIO alternate function to route TIM14_CH1
 * to PF0 instead of PF1. This enables PWM output on VALVE_CTRL1.
 * 
 * Hardware Multiplexing Process:
 * 1. Check if already configured for PF0 (no action needed)
 * 2. If PF1 was active, disable it (set as GPIO output low)
 * 3. Configure PF0 as alternate function AF2 (TIM14_CH1)
 * 4. Update active_valve state tracker
 * 
 * GPIO States:
 * - Before: PF1 = AF13/TIM14_CH1, PF0 = GPIO_OUT_LOW
 * - After:  PF0 = AF2/TIM14_CH1,  PF1 = GPIO_OUT_LOW
 * 
 * @note This function is called automatically by PWM_Set_Valve_Heater_Channels()
 * @note PWM duty cycle is controlled separately via __HAL_TIM_SET_COMPARE()
 * @note Function returns immediately if PF0 is already active (optimization)
 */
static void Switch_To_Valve1(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    if (active_valve == 1) return; // Already on valve 1
    
    // Disable PF1 PWM (set as input or output low)
    if (active_valve == 2)
    {
        GPIO_InitStruct.Pin = GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
    }
    
    // Enable PF0 as TIM14_CH1 (AF2)
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM14;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    
    active_valve = 1;
}

/**
 * @brief Switch TIM14_CH1 output to PF1 (VALVE_CTRL2)
 * 
 * Dynamically reconfigures GPIO alternate function to route TIM14_CH1
 * to PF1 instead of PF0. This enables PWM output on VALVE_CTRL2.
 * 
 * Hardware Multiplexing Process:
 * 1. Check if already configured for PF1 (no action needed)
 * 2. If PF0 was active, disable it (set as GPIO output low)
 * 3. Configure PF1 as alternate function AF13 (TIM14_CH1)
 * 4. Update active_valve state tracker
 * 
 * GPIO States:
 * - Before: PF0 = AF2/TIM14_CH1,  PF1 = GPIO_OUT_LOW
 * - After:  PF1 = AF13/TIM14_CH1, PF0 = GPIO_OUT_LOW
 * 
 * @note PF1 uses AF13 for TIM14_CH1 (different from PF0 which uses AF2)
 * @note This function is called automatically by PWM_Set_Valve_Heater_Channels()
 * @note PWM duty cycle is controlled separately via __HAL_TIM_SET_COMPARE()
 * @note Function returns immediately if PF1 is already active (optimization)
 */
static void Switch_To_Valve2(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    if (active_valve == 2) return; // Already on valve 2
    
    // Disable PF0 PWM (set as input or output low)
    if (active_valve == 1)
    {
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
    }
    
    // Enable PF1 as TIM14_CH1 (AF13)
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_TIM14;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    
    active_valve = 2;
}

/**
 * @brief Disable both valve heater outputs
 * 
 * Configures both PF0 and PF1 as GPIO outputs in low state, effectively
 * turning off both valve heaters. This is used when neither heater should
 * be active, typically during state transitions or shutdown.
 * 
 * Implementation:
 * 1. Configure PF0 as GPIO output, write LOW
 * 2. Configure PF1 as GPIO output, write LOW
 * 3. Clear active_valve state (set to 0)
 * 
 * GPIO Final State:
 * - PF0: GPIO_MODE_OUTPUT_PP, output = LOW
 * - PF1: GPIO_MODE_OUTPUT_PP, output = LOW
 * 
 * @note TIM14 continues running but has no output path (GPIO not in AF mode)
 * @note This provides a safe "all heaters off" state without stopping timers
 * @note Called automatically when PWM_Set_Valve_Heater_Channels(0, 0) is used
 */
static void Disable_Both_Valves(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Set both pins as outputs low
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
    
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
    
    active_valve = 0;
}

/* Public API Functions ------------------------------------------------------*/

/**
 * @brief Set PWM duty cycle for amplification heater channels
 * 
 * Controls the PWM output for both amplification heater elements using
 * independent hardware timers (TIM16 and TIM17). Both heaters can be
 * active simultaneously with different duty cycles.
 * 
 * Hardware Implementation:
 * - Channel 1 (PB6): TIM16_CH1N complementary output
 * - Channel 2 (PB7): TIM17_CH1N complementary output
 * 
 * @param ch1_duty PWM duty cycle for channel 1 (0-255)
 *                 - 0:   Heater off (0% duty cycle)
 *                 - 128: 50% duty cycle
 *                 - 255: Maximum heating (100% duty cycle)
 *                 Maps to PB6/AMP_CTRL1
 * 
 * @param ch2_duty PWM duty cycle for channel 2 (0-255)
 *                 - 0:   Heater off (0% duty cycle)
 *                 - 128: 50% duty cycle
 *                 - 255: Maximum heating (100% duty cycle)
 *                 Maps to PB7/AMP_CTRL2
 * 
 * Usage Examples:
 * @code
 * // Turn off both heaters
 * PWM_Set_Sample_Heater_Channels(0, 0);
 * 
 * // Channel 1 at 75%, Channel 2 off
 * PWM_Set_Sample_Heater_Channels(191, 0);
 * 
 * // Both active at different levels
 * PWM_Set_Sample_Heater_Channels(200, 150);
 * 
 * // Maximum heating on both
 * PWM_Set_Sample_Heater_Channels(255, 255);
 * @endcode
 * 
 * @note Both channels are independent - can be controlled simultaneously
 * @note Changes take effect immediately (no delay/buffering)
 * @note Safe to call at any frequency; no rate limiting needed
 * @note Duty cycle is linear: value/255 = duty cycle percentage
 * 
 * @warning High duty cycles generate significant heat - ensure proper cooling
 * @warning Application should implement PID control or limits for safety
 */
void PWM_Set_Sample_Heater_Channels(uint8_t ch1_duty, uint8_t ch2_duty)
{
    // Set TIM16 duty cycle for PB6 (AMP_CTRL1)
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, ch1_duty);
    
    // Set TIM17 duty cycle for PB7 (AMP_CTRL2)
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, ch2_duty);
}

/**
 * @brief Set PWM duty cycle for valve heater channels
 * 
 * Controls the PWM output for valve heater elements using GPIO multiplexing.
 * Due to hardware constraints (both pins share TIM14_CH1), only ONE valve
 * heater can be active at any given time. The function automatically handles
 * the GPIO alternate function switching.
 * 
 * Hardware Constraint:
 * - PF0 and PF1 both map to TIM14_CH1 (cannot output different PWM signals)
 * - Solution: Dynamic GPIO AF switching (multiplexing)
 * - Limitation: Only one valve heater active at a time
 * 
 * Multiplexing Logic:
 * -------------------
 * 1. ch1 > 0, ch2 = 0:  Enable PF0 only (switch GPIO AF to PF0)
 * 2. ch2 > 0, ch1 = 0:  Enable PF1 only (switch GPIO AF to PF1)
 * 3. Both > 0:          Priority to CH1 (PF0 active, PF1 disabled)
 * 4. Both = 0:          Both disabled (GPIO outputs low)
 * 
 * @param ch1_duty PWM duty cycle for channel 1 (0-255)
 *                 - Maps to PF0/VALVE_CTRL1
 *                 - 0:   Heater off
 *                 - 255: Maximum heating (100% duty cycle)
 * 
 * @param ch2_duty PWM duty cycle for channel 2 (0-255)
 *                 - Maps to PF1/VALVE_CTRL2
 *                 - 0:   Heater off
 *                 - 255: Maximum heating (100% duty cycle)
 * 
 * Usage Examples:
 * @code
 * // Turn off both valves
 * PWM_Set_Valve_Heater_Channels(0, 0);
 * 
 * // Valve 1 active at 75%
 * PWM_Set_Valve_Heater_Channels(191, 0);
 * 
 * // Valve 2 active at 60%
 * PWM_Set_Valve_Heater_Channels(0, 153);
 * 
 * // Both requested - CH1 takes priority (avoid this!)
 * PWM_Set_Valve_Heater_Channels(200, 150);  // Only CH1 will be active
 * @endcode
 * 
 * @note Application MUST ensure only one valve heater requested at a time
 * @note If both are non-zero, CH1 (PF0) takes priority
 * @note GPIO switching has ~microsecond latency (negligible for PID control)
 * @note Duty cycle changes take effect on next PWM period (~107 µs)
 * 
 * @warning Requesting both heaters simultaneously is not recommended
 * @warning Application should implement mutual exclusion logic
 * @warning High duty cycles generate significant heat - ensure proper cooling
 * 
 * @see Switch_To_Valve1() for PF0 AF configuration details
 * @see Switch_To_Valve2() for PF1 AF configuration details
 * @see Disable_Both_Valves() for shutdown logic
 */
void PWM_Set_Valve_Heater_Channels(uint8_t ch1_duty, uint8_t ch2_duty)
{
    if (ch1_duty > 0 && ch2_duty == 0)
    {
        // Only valve 1 active
        Switch_To_Valve1();
        __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, ch1_duty);
    }
    else if (ch2_duty > 0 && ch1_duty == 0)
    {
        // Only valve 2 active
        Switch_To_Valve2();
        __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, ch2_duty);
    }
    else if (ch1_duty > 0 && ch2_duty > 0)
    {
        // Both requested - give priority to valve 1
        // Application should avoid this scenario
        Switch_To_Valve1();
        __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, ch1_duty);
    }
    else
    {
        // Both zero - disable both valves
        Disable_Both_Valves();
    }
}