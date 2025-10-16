# PWM System Quick Reference Card

## Timer & Pin Assignments

| Function | GPIO | Timer | Channel | AF | Independent? |
|----------|------|-------|---------|----|--------------| 
| AMP_CTRL1 | PB6 | TIM16 | CH1N | AF2 | ✅ Yes |
| AMP_CTRL2 | PB7 | TIM17 | CH1N | AF2 | ✅ Yes |
| VALVE_CTRL1 | PF0 | TIM14 | CH1 | AF2 | ❌ Multiplexed |
| VALVE_CTRL2 | PF1 | TIM14 | CH1 | AF13 | ❌ Multiplexed |
| System Timing | - | TIM3 | - | - | N/A |

## Key Functions

```c
// Initialize (call once at startup)
PWM_Init();

// Amplification heaters (can both be active)
PWM_Set_Sample_Heater_Channels(ch1_duty, ch2_duty);

// Valve heaters (only one active at a time)
PWM_Set_Valve_Heater_Channels(ch1_duty, ch2_duty);
```

## PWM Parameters

- **Frequency:** 9.375 kHz
- **Resolution:** 8-bit (0-255)
- **Duty Cycle:** `(value / 255) × 100%`

| Value | Duty Cycle | Use Case |
|-------|------------|----------|
| 0 | 0% | Heater off |
| 64 | 25% | Low heat |
| 128 | 50% | Medium heat |
| 191 | 75% | High heat |
| 255 | 100% | Maximum heat |

## Usage Patterns

### ✅ Correct Usage

```c
// Amp heaters - both can be active
PWM_Set_Sample_Heater_Channels(200, 150);  // Both on

// Valve heaters - one at a time
PWM_Set_Valve_Heater_Channels(180, 0);     // CH1 only
PWM_Set_Valve_Heater_Channels(0, 200);     // CH2 only
PWM_Set_Valve_Heater_Channels(0, 0);       // Both off
```

### ❌ Incorrect Usage

```c
// DON'T: Request both valve heaters simultaneously
PWM_Set_Valve_Heater_Channels(180, 200);   // Only CH1 will be active!
```

## Hardware Constraints

### Amplification Heaters (PB6/PB7)
- ✅ Independent control
- ✅ Can run simultaneously
- ✅ Different duty cycles supported
- Uses TIM16_CH1N and TIM17_CH1N (complementary outputs)

### Valve Heaters (PF0/PF1)
- ⚠️ **Mutually exclusive** - only ONE active at a time
- ⚠️ Both share TIM14_CH1
- ⚠️ Application must enforce mutual exclusion
- Uses GPIO multiplexing to switch between pins

### USB Power Limiting
- 🔋 Automatically enabled when running on USB power
- 🔋 Limits combined PWM to 191 (75% of 255) to stay within USB specs
- 🔋 Proportionally scales both heaters if total exceeds limit
- 🔋 Maintains relative power ratio between heaters
- 🔋 See `USB_POWER_LIMITING.md` for details

## Troubleshooting

| Problem | Check |
|---------|-------|
| No PWM output | • Called `PWM_Init()`?<br>• Duty cycle > 0?<br>• Correct pin assignment? |
| Valve heater not switching | • Only one ch_duty > 0?<br>• Not requesting both simultaneously? |
| Wrong frequency | • Prescaler = 9?<br>• Period = 255? |

## Integration Example

```c
// In main initialization
PWM_Init();

// In PID control loop (called every 500ms)
void Update_PID(void) {
    // Compute PID outputs
    pid_controller_compute(SAMPLE_HEATER, temp);
    pid_controller_compute(VALVE_HEATER, temp);
    
    // Get PWM values
    uint8_t amp_pwm = pid_data[SAMPLE_HEATER].out;
    uint8_t valve_pwm = pid_data[VALVE_HEATER].out;
    
    // Update amplification heaters
    if (amp_ctrl_enabled) {
        if (high_side_mode) {
            PWM_Set_Sample_Heater_Channels(amp_pwm, 0);
        } else {
            PWM_Set_Sample_Heater_Channels(0, amp_pwm);
        }
    }
    
    // Update valve heaters (mutually exclusive)
    if (valve_ctrl_enabled) {
        if (high_side_mode) {
            PWM_Set_Valve_Heater_Channels(valve_pwm, 0);
        } else {
            PWM_Set_Valve_Heater_Channels(0, valve_pwm);
        }
    }
}
```

## References

- **Detailed Architecture:** See `PWM_ARCHITECTURE.md`
- **API Documentation:** See `Inc/io/pwm_init.h`
- **Implementation:** See `Src/io/pwm_init.c`
- **Integration Example:** See `Src/main.cpp` → `Update_PID()`

---

**Quick Tip:** When in doubt, remember:
- **Amp heaters** = Independent (both can be on)
- **Valve heaters** = Mutually exclusive (only one at a time)
