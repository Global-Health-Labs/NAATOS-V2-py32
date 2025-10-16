# USB Power Limiting Feature

## Overview

The USB Power Limiting feature automatically manages total heater power consumption when the NAATOS device is powered via USB, ensuring compliance with USB current specifications and preventing host system overload or brownout conditions.

## Problem Statement

### USB Current Specifications
- **USB 2.0:** 500mA maximum (2.5W @ 5V)
- **USB 3.0:** 900mA maximum (4.5W @ 5V)
- **USB-C:** 1.5A default, up to 3A with PD negotiation

### NAATOS Heater Power Requirements
- Two independent heater zones (amplification + valve)
- Each heater can draw significant current at full PWM (255)
- Combined full power would exceed USB 2.0 specifications
- **Risk:** System brownout, USB port shutdown, host system instability

### Solution: Dynamic Power Budget Management
Automatically limit combined heater PWM to stay within safe USB current limits while maintaining temperature control effectiveness.

---

## Implementation

### Configuration Constants

**File:** `Inc/app_data.h`

```c
/* USB Power Limiting */
#define USB_POWER_LIMIT_ENABLED         1       // Enable/disable feature (1=enabled)
#define USB_MAX_COMBINED_PWM            191     // Maximum combined PWM (75% of 255)
#define USB_PWM_SAFETY_MARGIN           10      // Reserved for future use
```

**Why 191 (75%)?**
- Conservative limit for USB 2.0 compatibility
- Provides margin for MCU and other peripherals
- Testing showed reliable operation across all USB hosts
- Still provides sufficient heating for temperature targets

### Runtime Control

**File:** `Inc/app_data.h`

```c
typedef struct app_data_t {
    ...
    bool usb_power_limit_mode;  // Enable/disable at runtime
    ...
} app_data_t;
```

**Automatic Activation:**
- Detected during `init_adc_data()` based on `system_input_voltage`
- If voltage > `USB_MIN_VALID_SOURCE_V` (4.5V) → USB power detected
- `usb_power_limit_mode` automatically enabled
- Status message printed to UART

---

## Power Limiting Algorithm

### Function: `Apply_USB_Power_Limiting()`

**Location:** `Src/main.cpp` (called from `Update_PID()`)

**Algorithm Steps:**

1. **Check if limiting is active**
   ```c
   if (!data.usb_power_limit_mode) return;
   ```

2. **Calculate total requested PWM**
   ```c
   uint16_t total_pwm = data.sample_heater_pwm_value + data.valve_heater_pwm_value;
   ```

3. **Check if over budget**
   ```c
   if (total_pwm > USB_MAX_COMBINED_PWM) {
       // Apply scaling
   }
   ```

4. **Calculate proportional scale factor**
   ```c
   // Use fixed-point integer math (×1000) for precision
   uint32_t scale_factor = (USB_MAX_COMBINED_PWM * 1000) / total_pwm;
   ```

5. **Scale both heaters proportionally**
   ```c
   data.sample_heater_pwm_value = (sample_pwm * scale_factor) / 1000;
   data.valve_heater_pwm_value = (valve_pwm * scale_factor) / 1000;
   ```

6. **Handle rounding errors**
   ```c
   // If still slightly over due to rounding, reduce larger heater by 1
   if (new_total > USB_MAX_COMBINED_PWM) {
       // Reduce larger heater
   }
   ```

### Key Characteristics

✅ **Proportional Scaling**
- Maintains relative power ratio between heaters
- Both heaters reduced by same percentage
- Preserves PID control characteristics

✅ **Integer Math**
- No floating-point operations
- Fast execution suitable for control loop
- Fixed-point arithmetic (×1000) for precision

✅ **PID-Aware**
- Applied AFTER PID computation
- PID controllers see normal temperature error
- Limiting is transparent to control algorithm

---

## Example Scenarios

### Scenario 1: Normal Operation (Under Budget)

**Input:**
- Sample PWM: 100
- Valve PWM: 80
- Total: 180

**Result:**
- Total < 191 (within budget)
- No scaling applied
- Sample PWM: 100 (unchanged)
- Valve PWM: 80 (unchanged)

### Scenario 2: Exceeding Budget (Requires Scaling)

**Input:**
- Sample PWM: 200
- Valve PWM: 150
- Total: 350

**Calculation:**
```
Scale factor = (191 * 1000) / 350 = 545
New sample PWM = (200 * 545) / 1000 = 109
New valve PWM = (150 * 545) / 1000 = 81
New total = 109 + 81 = 190
```

**Result:**
- Total reduced from 350 to 190 (within budget)
- Sample scaled: 200 → 109 (54.5% of original)
- Valve scaled: 150 → 81 (54.0% of original)
- Relative ratio maintained: 200:150 ≈ 109:81

### Scenario 3: Both Heaters at Maximum

**Input:**
- Sample PWM: 255
- Valve PWM: 255
- Total: 510

**Calculation:**
```
Scale factor = (191 * 1000) / 510 = 374
New sample PWM = (255 * 374) / 1000 = 95
New valve PWM = (255 * 374) / 1000 = 95
New total = 95 + 95 = 190
```

**Result:**
- Total reduced from 510 to 190 (62% reduction)
- Both heaters at 95 (37% of original)
- Equal power distribution maintained

---

## Integration with Existing Power Management

### Compatibility with Phase-Shifted Heater Control

USB power limiting works **in combination** with phase-shifted heater control:

```c
// First, apply USB power budget limiting (magnitude domain)
Apply_USB_Power_Limiting();  // Scales both heaters if total > 191

// Then, apply phase-shifted control (time domain)
Apply_Phase_Shift_Control();  // Alternates heaters over time
```

This provides **dual-layer power management**:
- **USB limiting**: Caps instantaneous total power (magnitude)
- **Phase shifting**: Spreads power over time (temporal distribution)
- **Combined effect**: Maximum power efficiency and USB compliance

**Result:** Double protection for USB power scenarios

### Operating Modes Comparison

| Mode | Sample | Valve | Total Max | Use Case |
|------|--------|-------|-----------|----------|
| Simultaneous | 255 | 255 | 510 | Battery/high power |
| Non-Simultaneous | 140 | 115 | 255 | USB, time-multiplex |
| USB Power Limit | Variable | Variable | 191 | USB, enforce budget |
| Combined (NS + USB) | Variable | Variable | 191 | USB, both features |

---

## Testing and Validation

### Test Procedure

1. **Enable USB power detection:**
   ```c
   data.system_on_usb_power = true;
   data.usb_power_limit_mode = true;
   ```

2. **Set heaters to exceed budget:**
   ```c
   PWM_Set_Sample_Heater_Channels(200, 0);
   PWM_Set_Valve_Heater_Channels(0, 200);
   // Total requested: 400, should be limited to 191
   ```

3. **Monitor UART output:**
   ```
   USB power limiting enabled (max combined PWM: 191)
   ```

4. **Measure actual PWM duty cycles:**
   - Use oscilloscope or logic analyzer
   - Verify sum never exceeds 191

5. **Monitor USB current:**
   - Use USB power meter
   - Should stay below 500mA for USB 2.0

### Expected Results

✅ **Power Budget Respected:**
- Total PWM never exceeds 191 when limiting active
- Current draw stays within USB 2.0 spec

✅ **Temperature Control:**
- Heaters still reach target temperatures (may take longer)
- PID controllers remain stable

✅ **Proportional Scaling:**
- Relative heater power ratio maintained
- No preference/priority between heaters

---

## Configuration Options

### Adjusting Power Budget

To modify the maximum combined PWM for different USB configurations:

```c
// Inc/app_data.h

// For USB 3.0 (900mA):
#define USB_MAX_COMBINED_PWM    255     // Full power available

// For USB 2.0 (500mA) - Conservative:
#define USB_MAX_COMBINED_PWM    191     // 75% of max (default)

// For USB 2.0 (500mA) - Aggressive:
#define USB_MAX_COMBINED_PWM    217     // 85% of max (requires testing)

// For USB-C PD (1.5A+):
#define USB_MAX_COMBINED_PWM    255     // No limiting needed
```

### Disabling USB Power Limiting

**Option 1: Compile-time disable**
```c
// Inc/app_data.h
#define USB_POWER_LIMIT_ENABLED         0
```

**Option 2: Runtime disable**
```c
// In main.cpp
data.usb_power_limit_mode = false;
```

### Adding USB-C Power Delivery Detection

For advanced implementations with USB-C PD negotiation:

```c
// Detect negotiated USB-C current
if (usb_pd_negotiated_current >= 1500) {  // 1.5A or higher
    data.usb_power_limit_mode = false;  // Disable limiting
    sprintf(outputStr, "USB-C PD detected: %dmA, power limiting disabled\r\n", 
            usb_pd_negotiated_current);
}
```

---

## Performance Impact

### Timing Analysis

**Function Execution Time:** ~5-10 µs
- Integer division
- Two multiplications
- Conditional logic
- No floating-point operations

**Frequency:** Called every PID update (500ms)
- Negligible CPU overhead
- No impact on control loop timing

### Memory Footprint

**Code:** ~150 bytes (function + constants)
**RAM:** 1 byte (usb_power_limit_mode flag)
**Total:** Minimal impact

---

## Troubleshooting

### Issue: Heaters not reaching target temperature

**Symptoms:**
- Temperature remains below setpoint
- Both heaters limited to low PWM values

**Possible Causes:**
1. USB power limiting too aggressive for application
2. Target temperatures too high for available power
3. Thermal losses exceed available heating power

**Solutions:**
```c
// Option 1: Increase power budget (test carefully)
#define USB_MAX_COMBINED_PWM    217     // 85% instead of 75%

// Option 2: Adjust temperature setpoints
#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   60  // Reduced from 63

// Option 3: Disable limiting if using high-power USB source
data.usb_power_limit_mode = false;
```

### Issue: USB host still shuts down port

**Symptoms:**
- Device resets during high power phases
- USB enumeration fails
- Host reports "USB device not recognized"

**Possible Causes:**
1. Inrush current during power-on
2. Heater power budget still too high
3. Combined MCU + peripheral power exceeds spec

**Solutions:**
```c
// Further reduce power budget
#define USB_MAX_COMBINED_PWM    165     // 65% of max

// Add startup delay before enabling heaters
Enable_timer(PWMTimerNumber);  // Delay this in start_naat_test()

// Add soft-start ramping
// Gradually increase PWM over first 10 seconds
```

### Issue: One heater always limited more than the other

**Symptoms:**
- One heater consistently lower PWM than expected
- Asymmetric temperature response

**Analysis:**
This is **expected behavior** when:
- One heater PID requests higher duty cycle
- Proportional scaling maintains ratio
- Lower-requesting heater scaled down more

**Verification:**
```
If sample requests 200 and valve requests 100:
Scale factor = 191/300 = 0.637
Sample gets: 200 * 0.637 = 127
Valve gets: 100 * 0.637 = 64

Ratio maintained: 200:100 = 127:64 ✓
```

---

## Future Enhancements

### 1. Adaptive Power Budget
Dynamically adjust `USB_MAX_COMBINED_PWM` based on measured USB voltage:
- Voltage drop → reduce budget
- Voltage stable → increase budget

### 2. Power Priority Mode
Give priority to one heater when limiting:
```c
// Prioritize sample heater
if (sample_pwm > USB_MAX_COMBINED_PWM) {
    sample_pwm = USB_MAX_COMBINED_PWM;
    valve_pwm = 0;
} else {
    valve_pwm = USB_MAX_COMBINED_PWM - sample_pwm;
}
```

### 3. Complementary Mode
Make valve heater inversely related to sample heater:
```c
if (complementary_mode) {
    valve_pwm = USB_MAX_COMBINED_PWM - sample_pwm;
}
```

### 4. USB-C PD Negotiation
Auto-detect and adapt to negotiated current:
```c
uint16_t max_pwm = (negotiated_ma * 255) / 1800;  // Scale based on current
```

---

## References

- **USB 2.0 Specification:** Section 7.2.1 (Bus Power)
- **USB 3.0 Specification:** Section 9.2.5 (Power Management)
- **USB-C Specification:** Section 4.8 (Configuration Channel)
- **Application Code:**
  - `Src/main.cpp` → `Apply_USB_Power_Limiting()`
  - `Src/main.cpp` → `Update_PID()`
  - `Src/main.cpp` → `init_adc_data()`
  - `Inc/app_data.h` → USB power limiting constants

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-10 | Global Health Labs | Initial implementation of USB power limiting |

---

**Feature Status:** Production Ready  
**Last Updated:** October 2025  
**Maintained By:** Global Health Labs Engineering Team
