# PWM Architecture Documentation for NAATOS V2

## Overview

This document describes the PWM (Pulse Width Modulation) architecture implemented for heater control in the NAATOS V2 system running on the PY32F003x8 microcontroller.

## Table of Contents

1. [Hardware Constraints](#hardware-constraints)
2. [Timer Allocation](#timer-allocation)
3. [GPIO Pin Mapping](#gpio-pin-mapping)
4. [Implementation Strategy](#implementation-strategy)
5. [PWM Specifications](#pwm-specifications)
6. [Usage Guidelines](#usage-guidelines)
7. [Troubleshooting](#troubleshooting)

---

## Hardware Constraints

The PY32F003x8 has limited timer peripherals compared to larger MCUs, which creates specific constraints for PWM generation:

### Available Timers
- **TIM1** - Advanced timer (16-bit)
- **TIM3** - General purpose timer (16-bit)
- **TIM14** - General purpose timer (16-bit)
- **TIM16** - General purpose timer (16-bit)
- **TIM17** - General purpose timer (16-bit)

**Note:** TIM2 is NOT available on PY32F003x8 (only on x6 variant)

### GPIO/Timer Mapping Limitations

#### Valve Heaters (PF0/PF1)
**Problem:** Both PF0 and PF1 can only access TIM14_CH1 (not independent channels)
- PF0 → TIM14_CH1 via AF2
- PF1 → TIM14_CH1 via AF13

**Impact:** Cannot generate two independent PWM signals on these pins simultaneously.

**Solution:** GPIO multiplexing - dynamically switch the alternate function to route TIM14_CH1 to either PF0 or PF1.

#### Amplification Heaters (PB6/PB7)
**Discovery:** PB6 and PB7 use complementary timer outputs (CH1N)
- PB6 → TIM16_CH1N via AF2
- PB7 → TIM17_CH1N via AF2

**Implication:** Complementary outputs (CH1N) are typically used for motor control with dead-time insertion, but they work perfectly as independent PWM outputs for this application.

---

## Timer Allocation

| Timer | Function | GPIO Pins | Channel Type | Frequency |
|-------|----------|-----------|--------------|-----------|
| TIM3  | System Timing | N/A | N/A | 1 kHz (1ms ticks) |
| TIM14 | Valve Heater PWM | PF0/PF1 (multiplexed) | CH1 | 9.375 kHz |
| TIM16 | Amp Heater 1 PWM | PB6 | CH1N (complementary) | 9.375 kHz |
| TIM17 | Amp Heater 2 PWM | PB7 | CH1N (complementary) | 9.375 kHz |

### Why These Allocations?

**TIM3 for System Timing:**
- Initially tried TIM1, but it conflicted with PWM requirements
- TIM3 provides reliable 1ms interrupts for system timing
- Keeps system timing independent from heater control

**TIM14 for Valve Heaters:**
- Only timer that can reach both PF0 and PF1
- Shared channel requires multiplexing strategy
- Application constraint: Only one valve heater active at a time

**TIM16/TIM17 for Amp Heaters:**
- Independent timers allow simultaneous operation
- Complementary outputs (CH1N) provide the necessary GPIO access
- Both heaters can run at different duty cycles concurrently

---

## GPIO Pin Mapping

### Amplification Heaters (Independent Control)

```
PB6 (AMP_CTRL1)
├── Timer: TIM16
├── Channel: CH1N (complementary output)
├── Alternate Function: AF2
└── Control: Independent PWM

PB7 (AMP_CTRL2)
├── Timer: TIM17
├── Channel: CH1N (complementary output)
├── Alternate Function: AF2
└── Control: Independent PWM
```

**Features:**
- ✅ Both can be active simultaneously
- ✅ Different duty cycles supported
- ✅ No multiplexing required
- ✅ Full 0-255 duty cycle range

### Valve Heaters (Multiplexed Control)

```
PF0 (VALVE_CTRL1)                PF1 (VALVE_CTRL2)
├── Timer: TIM14                 ├── Timer: TIM14
├── Channel: CH1                 ├── Channel: CH1
├── Alternate Function: AF2      ├── Alternate Function: AF13
└── Control: Multiplexed         └── Control: Multiplexed
        │                                 │
        └─────────── TIM14_CH1 ──────────┘
                (Shared resource)
```

**Features:**
- ⚠️ Only ONE can be active at a time
- ⚠️ Requires GPIO alternate function switching
- ✅ Full 0-255 duty cycle range when active
- ⚠️ Application must enforce mutual exclusion

---

## Implementation Strategy

### 1. GPIO Multiplexing for Valve Heaters

The valve heater control uses dynamic GPIO reconfiguration to route a single timer output to different pins.

**Switching Process:**
```c
// Switch to Valve 1 (PF0)
1. Check if already on PF0 (optimization)
2. If PF1 was active, set it to GPIO output LOW
3. Configure PF0 as AF2 (TIM14_CH1)
4. Update duty cycle via __HAL_TIM_SET_COMPARE()

// Switch to Valve 2 (PF1)
1. Check if already on PF1 (optimization)
2. If PF0 was active, set it to GPIO output LOW
3. Configure PF1 as AF13 (TIM14_CH1)
4. Update duty cycle via __HAL_TIM_SET_COMPARE()
```

**Timing:**
- GPIO reconfiguration: ~1-2 µs
- PWM period: ~107 µs (9.375 kHz)
- Negligible impact on control loops

### 2. Complementary Outputs for Amp Heaters

The amplification heaters use TIM16 and TIM17 complementary outputs (CH1N).

**Key Configuration Differences:**
```c
// Standard PWM:
HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_1);

// Complementary PWM:
HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);  // Note: PWMN function
HAL_TIMEx_PWMN_Start(&htim17, TIM_CHANNEL_1);

// Polarity Configuration:
sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;  // Match normal PWM behavior
```

---

## PWM Specifications

### Frequency Calculation
```
System Clock: 24 MHz
Prescaler: 9 (divide by 10)
Period: 255 (8-bit resolution)

Timer Clock = 24 MHz / (Prescaler + 1)
            = 24 MHz / 10
            = 2.4 MHz

PWM Frequency = Timer Clock / (Period + 1)
              = 2.4 MHz / 256
              = 9.375 kHz
```

### Resolution
- **8-bit:** 0-255 duty cycle values
- **Percentage:** (value / 255) × 100%
- **Examples:**
  - 0 = 0% (heater off)
  - 128 = 50.2% duty cycle
  - 191 = 74.9% duty cycle
  - 255 = 100% (maximum heating)

### Timing Characteristics
| Parameter | Value |
|-----------|-------|
| PWM Period | 106.7 µs |
| PWM Frequency | 9.375 kHz |
| Resolution | 256 levels (8-bit) |
| Minimum Pulse Width | ~417 ns (1/2.4MHz) |
| Maximum Duty Cycle | 100% (255/255) |

---

## Usage Guidelines

### Initialization

```c
#include "pwm_init.h"

int main(void) {
    // System initialization
    HAL_Init();
    SystemClock_Config();
    
    // Initialize PWM system
    PWM_Init();  // Must be called before using PWM functions
    
    // PWM channels are now ready, all heaters initially off
}
```

### Amplification Heater Control (Independent)

```c
// Both heaters can be controlled independently

// Turn on both heaters at different levels
PWM_Set_Sample_Heater_Channels(200, 150);  // CH1=78%, CH2=59%

// Run only heater 1
PWM_Set_Sample_Heater_Channels(180, 0);    // CH1=71%, CH2=off

// Run only heater 2
PWM_Set_Sample_Heater_Channels(0, 220);    // CH1=off, CH2=86%

// Turn off both
PWM_Set_Sample_Heater_Channels(0, 0);
```

### Valve Heater Control (Mutually Exclusive)

```c
// Only ONE valve heater can be active at a time

// ✅ Correct: Run valve 1 only
PWM_Set_Valve_Heater_Channels(180, 0);     // Valve 1 at 71%

// ✅ Correct: Switch to valve 2
PWM_Set_Valve_Heater_Channels(0, 200);     // Valve 2 at 78%

// ✅ Correct: Turn off both
PWM_Set_Valve_Heater_Channels(0, 0);

// ⚠️ Incorrect: Requesting both (CH1 takes priority)
PWM_Set_Valve_Heater_Channels(180, 200);   // Only valve 1 will be active!
```

### Best Practices

1. **Amplification Heaters:**
   - ✅ Can run simultaneously
   - ✅ Safe to update at any frequency
   - ✅ Independent duty cycle control
   - ⚠️ Implement PID control for temperature regulation

2. **Valve Heaters:**
   - ⚠️ Ensure only one heater requested at a time
   - ✅ Application should enforce mutual exclusion
   - ⚠️ Avoid switching between heaters too frequently (<1Hz recommended)
   - ✅ Always set one channel to 0 when switching

3. **General:**
   - ✅ Use PWM_Init() once during system startup
   - ✅ Changes take effect immediately (next PWM period)
   - ⚠️ High duty cycles generate significant heat - monitor temperatures
   - ✅ Safe to call PWM functions from interrupts (fast execution)

---

## Troubleshooting

### No PWM Output on Valve Heaters

**Symptoms:**
- No signal on PF0 or PF1
- Heater not heating

**Possible Causes:**
1. Both duty cycles set to 0
2. Both duty cycles non-zero (conflict - only CH1 active)
3. GPIO not properly initialized

**Solutions:**
```c
// Check that only one channel is active
PWM_Set_Valve_Heater_Channels(180, 0);  // ✅ Valve 1 only
PWM_Set_Valve_Heater_Channels(0, 200);  // ✅ Valve 2 only

// Verify PWM_Init() was called
PWM_Init();  // Must be called once at startup
```

### No PWM Output on Amplification Heaters

**Symptoms:**
- No signal on PB6 or PB7
- Heaters not heating

**Possible Causes:**
1. Duty cycle set to 0
2. Using wrong HAL function (standard PWM instead of complementary)
3. Timer not started

**Solutions:**
```c
// Verify non-zero duty cycle
PWM_Set_Sample_Heater_Channels(200, 150);  // Both active

// Check that PWMEx_PWMN functions are used (in pwm_init.c)
HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);  // ✅ Correct
// NOT: HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);  // ❌ Wrong!
```

### Incorrect PWM Frequency

**Expected:** ~9.375 kHz  
**If different:** Check prescaler and period settings

```c
// Correct configuration (in PWM_Init)
htim14.Init.Prescaler = 9;       // Divide by 10
htim14.Init.Period = 255;        // 8-bit resolution
```

### Valve Heater Switching Too Slow

**Symptoms:**
- Delay when switching between valve heaters
- Temperature control sluggish

**Explanation:**
- GPIO reconfiguration is fast (~1-2 µs)
- Thermal mass causes slow temperature response
- This is expected behavior, not a bug

**Not a problem for:**
- PID control loops (typically 10-1000 ms period)
- Temperature control (thermal time constants >> switching time)

---

## Code Examples

### Full Initialization Example

```c
#include "main.h"
#include "pwm_init.h"
#include "timers.h"

int main(void) {
    // Initialize HAL
    HAL_Init();
    
    // Configure system clock (24 MHz)
    SystemClock_Config();
    
    // Initialize GPIO
    GPIO_Init();
    
    // Initialize system timing (TIM3)
    TIMER_Init();
    
    // Initialize PWM system
    PWM_Init();
    
    // Start with all heaters off
    PWM_Set_Sample_Heater_Channels(0, 0);
    PWM_Set_Valve_Heater_Channels(0, 0);
    
    // Application loop
    while (1) {
        // Your control logic here
    }
}
```

### PID Control Integration Example

```c
void Update_PID(void) {
    // Compute PID for both heaters
    pid_controller_compute(SAMPLE_HEATER, data.sample_temperature_c);
    pid_controller_compute(VALVE_HEATER, data.valve_temperature_c);
    
    // Get PWM values from PID controllers
    data.sample_heater_pwm_value = pid_data[SAMPLE_HEATER].out;
    data.valve_heater_pwm_value = pid_data[VALVE_HEATER].out;
    
    // Update amplification heaters (can be independent)
    if (pwm_amp_ctrl.enabled) {
        if (pwm_amp_ctrl.heater_level_high) {
            // High-side control
            PWM_Set_Sample_Heater_Channels(data.sample_heater_pwm_value, 0);
        } else {
            // Low-side control
            PWM_Set_Sample_Heater_Channels(0, data.sample_heater_pwm_value);
        }
    }
    
    // Update valve heaters (mutually exclusive)
    if (pwm_valve_ctrl.enabled) {
        if (pwm_valve_ctrl.heater_level_high) {
            // High-side control
            PWM_Set_Valve_Heater_Channels(data.valve_heater_pwm_value, 0);
        } else {
            // Low-side control
            PWM_Set_Valve_Heater_Channels(0, data.valve_heater_pwm_value);
        }
    }
}
```

---

## Future Considerations

### Potential Enhancements

1. **Complementary Control Mode:**
   ```c
   // Make valve heater inversely related to sample heater
   if (complementary_mode) {
       valve_pwm = 255 - sample_pwm;
   }
   ```

2. **Power Budget Control:**
   ```c
   // Limit total power consumption
   #define MAX_COMBINED_PWM 191  // 75% max
   if (sample_pwm + valve_pwm > MAX_COMBINED_PWM) {
       // Scale down proportionally
   }
   ```

3. **Smooth Valve Switching:**
   ```c
   // Ramp down one heater before ramping up the other
   void Switch_Valve_Heater_Smooth(uint8_t from, uint8_t to) {
       // Gradual transition implementation
   }
   ```

---

## References

- **PY32F003x8 Datasheet:** GPIO alternate function tables
- **PY32F0xx HAL Driver:** Timer and GPIO APIs
- **Application Code:**
  - `Src/io/pwm_init.c` - PWM implementation
  - `Inc/io/pwm_init.h` - Public API
  - `Src/main.cpp` - PID integration example
  - `Src/timers.cpp` - System timing (TIM3)

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-10 | Global Health Labs | Initial documentation |

---

**Document Status:** Complete  
**Last Updated:** October 2025  
**Maintained By:** Global Health Labs Engineering Team
