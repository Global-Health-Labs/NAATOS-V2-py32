# Core Guidelines Advisory Report
Generated: 2025-10-09T18:27:19.005752Z

> This is an **advisory-only** report. No code was changed.

## app_data.h

**Function inventory:** 0 found
**Interrupt/Callback candidates:** none detected
**Potential file-scope globals (heuristic):** 56
- L111: `float setpoint;`
- L112: `float input;`
- L113: `float output;`
- L114: `float kp;`
- L115: `float ki;`
- L116: `float kd;`
- L117: `bool cold_temp_adjusted;`
- L118: `float cold_temp_offset_c;`
- L119: `};`
- L123: `bool test_active;`
- (+46 more)
**Magic numbers (rough count):** 36
**Samples:** 17, 5, 7, 12000, 4, 30, 150000, 8, 3, 6, 2, 83

**Advisories (no changes applied):**
- Prefer `constexpr` or `enum class` for domain constants; avoid magic numbers in control paths.
- Mark non-modified globals as `const` (or `constexpr`) where possible to express intent.
- For ownership, prefer `span`, references, or documented lifetime; avoid raw owning pointers.
- Keep ISR/Callback code minimal; defer work to non-interrupt context where possible.
- Add contract-style comments: preconditions/postconditions for public functions.

## main.h

**Function inventory:** 0 found
**Interrupt/Callback candidates:** none detected
**Potential file-scope globals (heuristic):** 40
- L24: `} heater_t;`
- L51: `bool    enabled;`
- L52: `bool    suspended;`
- L53: `bool    heater_level_high;`
- L54: `uint16_t pwm_state;`
- L55: `uint64_t pwm_bits[4];`
- L56: `uint8_t  pwm_tick_count;`
- L57: `} Pin_pwm_t;`
- L61: `GPIO_TypeDef    *GPIOx_AMP_TEMP_V;`
- L62: `uint16_t        GPIO_Pin_AMP_TEMP_V;`
- (+30 more)
**Magic numbers (rough count):** 1
**Samples:** 4

**Advisories (no changes applied):**
- Prefer `constexpr` or `enum class` for domain constants; avoid magic numbers in control paths.
- Mark non-modified globals as `const` (or `constexpr`) where possible to express intent.
- For ownership, prefer `span`, references, or documented lifetime; avoid raw owning pointers.
- Keep ISR/Callback code minimal; defer work to non-interrupt context where possible.
- Add contract-style comments: preconditions/postconditions for public functions.

## main.cpp

**Function inventory:** 104 found
- `__io_putchar()` — signature sample: `define UID_BASE_ADDR 0x1FFF0E00

int __io_putchar(int ch) {...`
- `print_UID()` — signature sample: `void print_UID(void)
{...`
- `for()` — signature sample: `if 0    
    for (int i=0; i<16; i++)
    {...`
- `ReadFlag()` — signature sample: `define FLAG_VALUE    0xDEADBEEF  

uint32_t ReadFlag(void)
{...`
- `WriteFlag()` — signature sample: `void WriteFlag(uint32_t value)
{...`
- `SetOptionBytes()` — signature sample: `ifdef SET_OB_ONCE
void SetOptionBytes(void)
{...`
- `PrintOptionBytes()` — signature sample: `endif 



void PrintOptionBytes(void)
        {...`
- `if()` — signature sample: `if (OBInit.USERConfig & OB_USER_BOR_EN) {...`
- `switch()` — signature sample: `switch (OBInit.USERConfig & FLASH_OPTR_BOR_LEV) {...`
- `pid_init()` — signature sample: `void pid_init(heater_t heater, CONTROL pid_settings){...`
- `if()` — signature sample: `if (data.cold_ambient_temp_mode && pid_settings.cold_temp_adjusted) {...`
- `system_setup()` — signature sample: `void system_setup() {...`
- (+92 more)

**Interrupt/Callback candidates:** none detected
**Potential file-scope globals (heuristic):** 371
- L31: `UART_HandleTypeDef UartHandle;`
- L32: `GPIO_InitTypeDef GpioInitStruct;`
- L33: `GPIO_InitTypeDef AdcPinStruct;`
- L35: `Pin_assignments_t Pins;`
- L37: `app_data_t data;`
- L38: `flags_t flags;`
- L40: `int8_t PWMTimerNumber;`
- L41: `int8_t LEDTimerNumber;`
- L42: `int8_t PIDTimerNumber;`
- L43: `int8_t MinuteTimerNumber;`
- (+361 more)
**Magic numbers (rough count):** 105
**Samples:** 4, 5, 25, 75, 2, 00, 250, -12, 256, 15, 14, 13

**Advisories (no changes applied):**
- Prefer `constexpr` or `enum class` for domain constants; avoid magic numbers in control paths.
- Mark non-modified globals as `const` (or `constexpr`) where possible to express intent.
- For ownership, prefer `span`, references, or documented lifetime; avoid raw owning pointers.
- Keep ISR/Callback code minimal; defer work to non-interrupt context where possible.
- Add contract-style comments: preconditions/postconditions for public functions.
