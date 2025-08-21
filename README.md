# NAATOS-V2-py32
This NAATOS V2 firmware project is designed for the Puya PY32F003 series ARM Cortex M0+ processors.  
MK series (MK5 / MK6) development is in the PY32_MK subdirectory. The other subdirectories were for initial devkit based prototyping and are not currently maintained.

## Building the project
IDE / Build environment: Visual Studio / Make

On Windows: $make -f Makefile_windows
On Linux: $make Makefile

/PY32_enviornment_setup/install_puya32_windows_env.bat
	- included to help configure enviornment for Windows


## Device programming
Segger J-Link BASE Compact (or equivalent). Low cost ARM JTAG programmers will likely also work.  
MK5 / MK6 boards have a 6 pin TAG Connect header. This contains the programming pins as well as the UART tx output for debug and test.   
The TAG Connect 2030-IDC-NL cable can plug into 20 pin ARM JTAG programming connector via the following adapter:  
https://www.tag-connect.com/product/arm20-ctx-20-pin-to-tc2030-idc-adapter-for-cortex  


## Edit Makefile
Change the settings in Makefile

* **MCU_TYPE** The MCU type you are using
* **USE_LL_LIB** Puya provides two sets of library, HAL and LL, set `USE_LL_LIB ?= y` to use LL instead of HAL.
  * No LL Library for PY32F07x
* **ENABLE_PRINTF_FLOAT** set it to `y` to `-u _printf_float` to link options. This will increase the binary size.
* **USE_FREERTOS** Set `USE_FREERTOS ?= y` will include FreeRTOS in compilation
* **USE_DSP** Include CMSIS DSP or not
* **FLASH_PROGRM**
  * If you use J-Link, `FLASH_PROGRM` can be jlink or pyocd
  * If you use DAPLink, set `FLASH_PROGRM ?= pyocd`
  * ST-LINK is not supported yet.
* **ARM_TOOLCHAIN** Make sure it points to the correct path of arm-none-eabi-gcc

## Compiling And Flashing

```bash
# clean source code
make clean
# build
make
# or make with verbose output
V=1 make
# flash
make flash