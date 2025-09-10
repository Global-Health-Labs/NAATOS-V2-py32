# NAATOS-V2-py32
This NAATOS V2 firmware project is designed for the Puya PY32F003 series ARM Cortex M0+ processors.  
MK series (MK5 / MK6) development is in the PY32_MK subdirectory. The other subdirectories were for initial devkit based prototyping and are not currently maintained.

## Building the project
IDE / Build environment: Visual Studio - WindowsOS

## Device programming
Segger J-Link BASE Compact (or equivalent). Low cost ARM JTAG programmers will likely also work.  
MK5 / MK6 boards have a 6 pin TAG Connect header. This contains the programming pins as well as the UART tx output for debug and test.   
The TAG Connect 2030-IDC-NL cable can plug into 20 pin ARM JTAG programming connector via the following adapter:  
https://www.tag-connect.com/product/arm20-ctx-20-pin-to-tc2030-idc-adapter-for-cortex  
