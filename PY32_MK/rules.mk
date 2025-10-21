################################################################################
# Build Rules and Compiler Configuration for PY32 Projects
# 
# Description:
#   Core build rules, compiler flags, and targets for PY32 microcontroller
#   projects. This file is included by Makefile_windows and handles the
#   actual compilation, linking, and flashing process.
#
# Targets:
#   all       - Build all output files (elf, bin, hex, lst)
#   clean     - Remove all build artifacts
#   flash     - Program firmware to device via JLink or PyOCD
#   echo      - Debug: print internal makefile variables
#
# Variables (set in Makefile_windows):
#   ARM_TOOLCHAIN  - Path to ARM GCC toolchain
#   BUILD_DIR      - Output directory for build artifacts
#   PROJECT        - Project name (output file prefix)
#   CDIRS/CFILES   - C source directories and files
#   ADIRS/AFILES   - Assembly source directories and files
#   INCLUDES       - Include directory paths
#   LIB_FLAGS      - Compiler defines (MCU type, HAL/LL selection)
#   LDSCRIPT       - Linker script path
#   FLASH_PROGRM   - Programmer selection (jlink or pyocd)
#
################################################################################

##### Verbosity Control #####

# Verbose build output control
# 'make V=1' will show all compiler calls (useful for debugging build issues)
# 'make V=0' or just 'make' will show simplified output
V		?= 0
ifeq ($(V),0)
Q		:= @
NULL	:= 2>/dev/null
endif

##### Toolchain Executable Names #####

# ARM GCC cross-compiler toolchain prefix
# Creates: arm-none-eabi-gcc, arm-none-eabi-g++, arm-none-eabi-as, etc.
PREFIX		?= $(ARM_TOOLCHAIN)/arm-none-eabi-

# Toolchain components
CC		= $(PREFIX)gcc         # C compiler
XX		= $(PREFIX)g++         # C++ compiler
AS		= $(PREFIX)as          # Assembler
LD		= $(PREFIX)ld          # Linker (typically not used directly)
OBJCOPY		= $(PREFIX)objcopy     # Binary format converter (elf -> bin/hex)
OBJDUMP		= $(PREFIX)objdump     # Disassembler and symbol dumper

# Project directory structure
TOP		= .                    # Top-level project directory
BDIR		= $(TOP)/$(BUILD_DIR)  # Build output directory

##### Source File Collection #####

# Collect all C source files from specified directories
# For each directory in CDIRS, find all .c files (non-recursive, -maxdepth 1)
CSOURCES 	:= $(foreach dir, $(CDIRS), $(shell find $(TOP)/$(dir) -maxdepth 1 -name '*.c'))

# Add individual C source files specified in CFILES
CSOURCES 	+= $(addprefix $(TOP)/, $(CFILES))

# Collect all C++ source files from specified directories
CPPSOURCES	:= $(foreach dir, $(CDIRS), $(shell find $(TOP)/$(dir) -maxdepth 1 -name '*.cpp'))

# Add individual C++ source files specified in CPPFILES
CPPSOURCES 	+= $(addprefix $(TOP)/, $(CPPFILES))

# Collect all assembly source files from specified directories
ASOURCES := $(foreach dir, $(ADIRS), $(shell find $(TOP)/$(dir) -maxdepth 1 -name '*.s'))

# Add individual assembly source files specified in AFILES (e.g., startup files)
ASOURCES += $(addprefix $(TOP)/, $(AFILES))

##### Object and Dependency File Generation #####

# Convert source file paths to object file paths in build directory
# Preserves source directory structure within BUILD_DIR
# Example: Src/main.c -> Build/Src/main.o
OBJS = $(CSOURCES:$(TOP)/%.c=$(BDIR)/%.o)
OBJS += $(CPPSOURCES:$(TOP)/%.cpp=$(BDIR)/%.o)
OBJS += $(ASOURCES:$(TOP)/%.s=$(BDIR)/%.o)

# Dependency files (.d) for automatic header file change detection
# Allows make to rebuild only when included headers change
DEPS=$(CSOURCES:$(TOP)/%.c=$(BDIR)/%.d)

##### Compiler and Linker Flags #####

# Architecture and CPU-specific flags
# Cortex-M0+ is the CPU core used in PY32F0xx series
ARCH_FLAGS	:= -mcpu=cortex-m0plus

# Debug information format selection
# -g0:      No debug info (smallest binary)
# -g/-g2:   Default debug info
# -g3:      Maximum debug info (includes macro definitions)
# -gdwarf-3: DWARF version 3 format (compatible with most debuggers)
# -gdwarf-5: Latest DWARF format (better but requires newer tools)
# https://gcc.gnu.org/onlinedocs/gcc-12.2.0/gcc/Debugging-Options.html
DEBUG_FLAGS ?= -gdwarf-3

# Optimization level
# -O0: No optimization (fastest compile, largest/slowest code, best for debugging)
# -O1: Basic optimization
# -O2: Moderate optimization (good balance)
# -O3: Aggressive optimization (may increase code size)
# -Os: Optimize for size (recommended for embedded systems)
OPT		?= -Os

# C compiler flags
# -std=c99:             Use C99 standard
# -D$(LIB_FLAGS):       Define MCU type and HAL/LL selection macros
# -Wall:                Enable all common warnings
# -ffunction-sections:  Place each function in its own section (enables --gc-sections)
# -fdata-sections:      Place each data item in its own section
TGT_CFLAGS	?= $(ARCH_FLAGS) $(DEBUG_FLAGS) $(OPT) -std=c99 $(addprefix -D, $(LIB_FLAGS)) -Wall -ffunction-sections -fdata-sections

# C++ compiler flags
# -std=c++11:           Use C++11 standard (needed for NAATOS V2)
TGT_CPPFLAGS	?= $(ARCH_FLAGS) $(DEBUG_FLAGS) $(OPT) -std=c++11 $(addprefix -D, $(LIB_FLAGS)) -Wall -ffunction-sections -fdata-sections

# Assembly compiler flags
# -Wa,--warn:           Pass --warn flag to assembler (enable warnings)
TGT_ASFLAGS	?= $(ARCH_FLAGS) $(DEBUG_FLAGS) $(OPT) -Wa,--warn

# Linker flags
# -specs=nano.specs:      Use newlib-nano (smaller C library for embedded)
# -specs=nosys.specs:     No syscalls (bare-metal environment)
# -lc -lm:                Link standard C library and math library
# -Wl,-Map=...:           Generate map file showing memory layout
# -Wl,--gc-sections:      Remove unused sections (reduces binary size)
# -Wl,--print-memory-usage: Print memory usage summary after linking
TGT_LDFLAGS	?= $(ARCH_FLAGS) -specs=nano.specs -specs=nosys.specs -lc -lm \
				-Wl,-Map=$(BDIR)/$(PROJECT).map \
				-Wl,--gc-sections \
				-Wl,--print-memory-usage

# Suppress RWX segment warnings in GCC 12+ (common in embedded linker scripts)
GCC_VERSION := $(shell $(CC) -dumpversion)
IS_GCC_ABOVE_12 := $(shell expr "$(GCC_VERSION)" ">=" "12")
ifeq "$(IS_GCC_ABOVE_12)" "1"
    TGT_LDFLAGS += -Wl,--no-warn-rwx-segments
endif

# Arch and target specified flags
ARCH_FLAGS	:= -mcpu=cortex-m0plus
# Debug options, -gdwarf-2 for debug, -g0 for release 
# https://gcc.gnu.org/onlinedocs/gcc-12.2.0/gcc/Debugging-Options.html
#  -g: system’s native format, -g0:off, -g/g1,-g2,-g3 -> more verbosely
#  -ggdb: for gdb, -ggdb0:off, -ggdb/ggdb1,-ggdb2,-ggdb3 -> more verbosely
#  -gdwarf: in DWARF format, -gdwarf-2,-gdwarf-3,-gdwarf-4,-gdwarf-5
DEBUG_FLAGS ?= -gdwarf-3

OPT		?= -Os
# C flags
TGT_CFLAGS	?= $(ARCH_FLAGS) $(DEBUG_FLAGS) $(OPT) -std=c99 $(addprefix -D, $(LIB_FLAGS)) -Wall -ffunction-sections -fdata-sections
# C++ flags
TGT_CPPFLAGS	?= $(ARCH_FLAGS) $(DEBUG_FLAGS) $(OPT) -std=c++11 $(addprefix -D, $(LIB_FLAGS)) -Wall -ffunction-sections -fdata-sections
# ASM flags
TGT_ASFLAGS	?= $(ARCH_FLAGS) $(DEBUG_FLAGS) $(OPT) -Wa,--warn
# LD flags
TGT_LDFLAGS	?= $(ARCH_FLAGS) -specs=nano.specs -specs=nosys.specs -lc -lm \
				-Wl,-Map=$(BDIR)/$(PROJECT).map \
				-Wl,--gc-sections \
				-Wl,--print-memory-usage

GCC_VERSION := $(shell $(CC) -dumpversion)
IS_GCC_ABOVE_12 := $(shell expr "$(GCC_VERSION)" ">=" "12")
ifeq "$(IS_GCC_ABOVE_12)" "1"
    TGT_LDFLAGS += -Wl,--no-warn-rwx-segments
endif

# Enable floating-point printf support if requested
# Adds ~5-10KB to binary but allows %f formatting in printf/sprintf
ifeq ($(ENABLE_PRINTF_FLOAT),y)
TGT_LDFLAGS	+= -u _printf_float
endif

# Convert include directory list to compiler -I flags
TGT_INCFLAGS := $(addprefix -I $(TOP)/, $(INCLUDES))


################################################################################
# Build Targets
################################################################################

.PHONY: all clean flash echo

# Default target: build all output files
all: fullcheck $(BDIR)/$(PROJECT).elf $(BDIR)/$(PROJECT).bin $(BDIR)/$(PROJECT).hex $(BDIR)/$(PROJECT).lst

# Pre-build validation checks
fullcheck:
	@if [ '$(findstring PY32F07,$(MCU_TYPE))' = 'PY32F07' ] && [ '$(USE_LL_LIB)' = 'y' ]; then \
		echo "LL for PY32F07x is not supported yet"; \
		return 1; \
	fi

# Debug target: print makefile variables for troubleshooting
echo:
	$(info 1. $(AFILES))
	$(info 2. $(ASOURCES))
	$(info 3. $(CSOURCES))
	$(info 4. $(OBJS))
	$(info 5. $(TGT_INCFLAGS))

################################################################################
# Dependency Management
################################################################################

# Include dependency files (.d) for automatic rebuilding when headers change
# The '-' prefix suppresses errors if .d files don't exist yet (first build)
-include $(DEPS)

################################################################################
# Compilation Rules
################################################################################

# Compile C source files to object files
# Pattern: Build/Src/main.o from Src/main.c
# -MT: Set target name in dependency file
# -MD: Generate dependency file
# -MF: Specify dependency file name
# -MP: Add phony targets for each dependency (prevents errors if header is deleted)
$(BDIR)/%.o: %.c
	@printf "  CC\t$<\n"
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(TGT_CFLAGS) $(TGT_INCFLAGS) -MT $@ -o $@ -c $< -MD -MF $(BDIR)/$*.d -MP

# Compile C++ source files to object files
$(BDIR)/%.o: %.cpp
	@printf "  XX\t$<\n"
	@mkdir -p $(dir $@)
	$(Q)$(XX) $(TGT_CPPFLAGS) $(TGT_INCFLAGS) -MT $@ -o $@ -c $< -MD -MF $(BDIR)/$*.d -MP

# Compile assembly source files to object files
$(BDIR)/%.o: %.s
	@printf "  AS\t$<\n"
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(TGT_ASFLAGS) -o $@ -c $<

################################################################################
# Linking and Binary Generation
################################################################################

# Link all object files into final ELF executable
# ELF contains debug symbols and is used for debugging
$(BDIR)/$(PROJECT).elf: $(OBJS) $(TOP)/$(LDSCRIPT)
	@printf "  LD\t$(LDSCRIPT) -> $@\n"
	$(Q)$(CC) $(TGT_LDFLAGS) -T$(TOP)/$(LDSCRIPT) $(OBJS) -o $@

# Convert ELF to raw binary format (for flashing)
# Binary file contains only program code/data, no debug info
%.bin: %.elf
	@printf "  OBJCP BIN\t$@\n"
	$(Q)$(OBJCOPY) -I elf32-littlearm -O binary  $< $@

# Convert ELF to Intel HEX format (alternative flash format)
# HEX format includes address information, useful for some programmers
%.hex: %.elf
	@printf "  OBJCP HEX\t$@\n"
	$(Q)$(OBJCOPY) -I elf32-littlearm -O ihex  $< $@

# Create assembly listing with source code interleaved
# Useful for debugging and understanding compiler optimizations
%.lst: %.elf
	@printf "  OBJDP LST\t$@\n"
	$(Q)$(OBJDUMP) --source $< > $@

################################################################################
# Utility Targets
################################################################################

# Clean all build artifacts
clean:
	rm -rf $(BDIR)/*

# Flash firmware to device using selected programmer
# Requires built ELF file
# FLASH_PROGRM can be 'jlink' or 'pyocd' (set in Makefile_windows)
flash: $(BDIR)/$(PROJECT).elf
ifeq ($(FLASH_PROGRM),jlink)
	# Flash using SEGGER J-Link
	# -device: Target MCU (uppercase, e.g., PY32F003X8)
	# -if swd: Use Serial Wire Debug interface
	# -speed 4000: Communication speed in kHz
	# -JLinkScriptFile: Custom J-Link script for device-specific setup
	# -CommanderScript: Commands to execute (erase, program, verify, reset)
	$(JLINKEXE) -device $(JLINK_DEVICE) -if swd -speed 4000 -JLinkScriptFile $(TOP)/Misc/jlink-script -CommanderScript $(TOP)/Misc/jlink-command
else ifeq ($(FLASH_PROGRM),pyocd)
	# Flash using PyOCD (open-source alternative)
	# First erase the chip, then load the firmware
	# -t: Target device (lowercase, e.g., py32f003x8)
	# --chip: Erase entire chip (not just used sectors)
	# --config: PyOCD configuration YAML file
	$(PYOCD_EXE) erase -t $(PYOCD_DEVICE) --chip --config $(TOP)/Misc/pyocd.yaml
	$(PYOCD_EXE) load $< -t $(PYOCD_DEVICE) --config $(TOP)/Misc/pyocd.yaml
else
	@echo "FLASH_PROGRM is invalid (must be 'jlink' or 'pyocd')\n"
endif
