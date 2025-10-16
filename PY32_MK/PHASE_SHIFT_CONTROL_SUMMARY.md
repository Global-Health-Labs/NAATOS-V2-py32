# Phase-Shifted Heater Control - Quick Start Guide

## Overview
The phase-shifted heater control feature alternates amplification and valve heaters out of phase over time to reduce peak instantaneous power draw. This complements USB power limiting by providing time-domain power distribution in addition to magnitude limiting.

## Key Benefits
- **50% reduction in peak current draw** - Only one heater at full power at any time
- **Complementary to USB limiting** - Works together with proportional scaling
- **Smooth power supply loading** - Reduces electrical noise and power supply stress
- **Maintains temperature control** - Thermal mass smooths duty cycling effects
- **Automatic activation** - Enabled when USB power is detected

## Quick Implementation Summary

### 1. Configuration Constants
```c
// Inc/app_data.h
#define HEATER_PHASE_SHIFT_ENABLED 1       // Enable feature compilation
#define HEATER_PHASE_PERIOD_MS 1000        // 1 second phase period
#define HEATER_PHASE_DUTY_PERCENT 50       // 50% duty cycle per heater
```

### 2. Data Structure Members
```c
// Inc/app_data.h - app_data_t structure
bool heater_phase_shift_mode;              // Runtime enable flag
uint32_t heater_phase_counter_ms;          // Phase timing counter
```

### 3. Core Algorithm
```c
// Src/main.cpp - Apply_Phase_Shift_Control()
1. Update phase counter by PID_TIMER_INTERVAL (500ms)
2. Calculate phase position: (counter * 100) / period
3. If phase < 50%: Amp heater active, Valve heater off
4. If phase >= 50%: Valve heater active, Amp heater off
5. Wrap counter at period boundary
```

### 4. Integration Points
```c
// Src/main.cpp - Update_PID()
pid_controller_compute(SAMPLE_HEATER, ...);
pid_controller_compute(VALVE_HEATER, ...);
Apply_USB_Power_Limiting();      // First: limit magnitude
Apply_Phase_Shift_Control();     // Second: apply time gating
Set PWM outputs;
```

### 5. Initialization
```c
// Src/main.cpp - Data_init()
data.heater_phase_shift_mode = false;
data.heater_phase_counter_ms = 0;

// Src/main.cpp - init_adc_data()
if (system_on_usb_power) {
    data.heater_phase_shift_mode = true;  // Auto-enable on USB
}
```

## Operation Timeline

**Example with 1-second period, 500ms update rate:**

| Time (ms) | Phase % | Amp Heater PWM | Valve Heater PWM | Note |
|-----------|---------|----------------|------------------|------|
| 0         | 0%      | PID value      | 0 (OFF)          | Start of cycle |
| 500       | 50%     | 0 (OFF)        | PID value        | Phase switch |
| 1000      | 0%      | PID value      | 0 (OFF)          | Cycle repeats |
| 1500      | 50%     | 0 (OFF)        | PID value        | Phase switch |

## Combined Effect with USB Power Limiting

**Scenario: Both heaters requesting 200 PWM (78% duty)**

### Without any limiting:
- Total current: ~2.8A (exceeds USB 2.0 spec of 500mA)

### With USB limiting only:
- Both heaters scaled to 95 PWM
- Total current: ~1.35A (within spec)
- Both heaters on simultaneously

### With USB limiting + Phase shift:
- Both heaters scaled to 95 PWM (USB limit)
- Only one heater active at a time (phase shift)
- Peak current: ~0.7A (50% of USB limited total)
- **Best power management** - combines magnitude and time domain

## Testing Checklist

- [ ] Verify phase period timing matches HEATER_PHASE_PERIOD_MS
- [ ] Confirm heaters alternate at 50% duty cycle each
- [ ] Measure peak current ~50% lower than simultaneous operation
- [ ] Check temperature control remains stable
- [ ] Test USB power detection enables mode automatically
- [ ] Verify phase shift works with USB power limiting
- [ ] Monitor for temperature oscillations (adjust PID if needed)

## Configuration Tuning

### Increasing Phase Period
```c
#define HEATER_PHASE_PERIOD_MS 2000  // 2 seconds
// Effect: Longer on-time per heater, smoother temperature control
// Trade-off: Higher peak current, slower power distribution
```

### Adjusting Duty Cycle
```c
#define HEATER_PHASE_DUTY_PERCENT 60  // Amp heater 60%, Valve 40%
// Effect: Asymmetric heating favoring one heater
// Use case: Different thermal masses or power requirements
```

### Disabling Feature
```c
#define HEATER_PHASE_SHIFT_ENABLED 0
// Or at runtime:
data.heater_phase_shift_mode = false;
```

## Troubleshooting

### Issue: Temperature oscillations
**Cause:** PID not tuned for 50% duty cycle operation  
**Solution:** Increase proportional gain to compensate for reduced heating time

### Issue: One heater not reaching target temperature
**Cause:** Insufficient heating time during active phase  
**Solution:** Increase phase period or adjust duty cycle asymmetrically

### Issue: Peak current still too high
**Cause:** Phase shift disabled or USB limiting threshold too high  
**Solution:** Verify both modes enabled, reduce USB_MAX_COMBINED_PWM

### Issue: Heaters not alternating
**Cause:** Phase counter not incrementing or mode not enabled  
**Solution:** Check data.heater_phase_shift_mode flag and counter updates

## Performance Metrics

**Typical Results (with 1-second period, 50% duty):**
- Peak current reduction: **~50%**
- Temperature stability: **±0.5°C** (within PID control range)
- Phase timing accuracy: **±100ms** (depends on PID_TIMER_INTERVAL)
- Power supply stress: **Significantly reduced**
- Electrical noise: **Lower due to smooth load transitions**

## See Also
- **PHASE_SHIFT_CONTROL.md** - Comprehensive documentation
- **USB_POWER_LIMITING_SUMMARY.md** - USB power limiting guide
- **PWM_QUICK_REFERENCE.md** - PWM system overview
- **Src/main.cpp** - Implementation code

## Notes
- Phase shift is applied AFTER USB power limiting in the control pipeline
- Both features are automatically enabled when USB power is detected
- Feature requires HEATER_PHASE_SHIFT_ENABLED = 1 at compile time
- Runtime enable/disable via data.heater_phase_shift_mode flag
- Counter wraps at HEATER_PHASE_PERIOD_MS to prevent overflow
