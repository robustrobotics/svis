#************************************************************************
# Configuration
#************************************************************************
M_PROJECT := teensy3-blinky

M_CPU_CLOCK := 72000000
M_USB_TYPE := USB_SERIAL
M_ARDUINO_VERSION := 10613
M_TEENSYDUINO_VERSION := 132
M_OPT_N_WARN := -Wall -Os

PROJECT_PATH = /home/fla/svis

SRC_PATH := src
BUILD_PATH := build
BIN_PATH := $(BUILD_PATH)/bin
$(info BIN_PATH: ${BIN_PATH})
TEENSY3_COPY_PATH := $(BUILD_PATH)/teensy3
$(info TEENSY3_COPY_PATH: ${TEENSY3_COPY_PATH})
PWD := $(realpath $(shell pwd))

TOOLSPATH := $(realpath $(PROJECT_PATH)/teensy/tools)
# LIBRARYPATH  := $(realpath $(PROJECT_PATH)/libraries)
COMPILERPATH := /home/fla/gcc-arm-none-eabi-4_8-2014q3/bin

#************************************************************************
# Definitions
#************************************************************************

# Setting up linker definitions and other defines
CPPFLAGS = $(M_OPT_N_WARN) -mcpu=cortex-m4 -mthumb -nostdlib -MMD \
	-DF_CPU=$(M_CPU_CLOCK) -D__MK20DX256__ -D$(M_USB_TYPE) -DLAYOUT_US_ENGLISH \
	-DARDUINO=$(M_ARDUINO_VERSION) -DTEENSYDUINO=$(M_TEENSYDUINO_VERSION) \
	-DTEENSY_VERSION=3.2 -DTEENSY_BOARD=TEENSY31
CXXFLAGS = -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti
ASMFLAGS = -x assembler-with-cpp
CFLAGS =
LDFLAGS = -O -Wl,--gc-sections,--relax,--defsym=__rtc_localtime=$(shell date '+%s') \
	-mcpu=cortex-m4 -mthumb --specs=nano.specs
LIBS = -larm_cortexM4l_math -lm

# Intermediate library for Teensy3 core
LIBTEENSY3 := $(BUILD_PATH)/libteensy3.a

# Tools needed
AR := $(abspath $(COMPILERPATH))/arm-none-eabi-ar
CC := $(abspath $(COMPILERPATH))/arm-none-eabi-gcc
CXX := $(abspath $(COMPILERPATH))/arm-none-eabi-g++
OBJCOPY := $(abspath $(COMPILERPATH))/arm-none-eabi-objcopy
SIZE := $(abspath $(COMPILERPATH))/arm-none-eabi-size

#************************************************************************
# Make targets
#************************************************************************

# (Public) target definitions ahead
.PHONY: all clean upload

# Same as build
all: $(BIN_PATH)/$(M_PROJECT).hex

clean:
	@rm -rf $(BUILD_PATH)

upload: all
	$(TOOLSPATH)/teensy_loader_cli/teensy_loader_cli -mmcu=mk20dx256 -wv $(BIN_PATH)/$(M_PROJECT).hex

# Some functions
relpath2        = $(subst $(1)/,,$(2))
relpath         = $(subst $(PWD)/,,$(1))
list_files      = $(realpath $(shell find $(1) -type f))
filter_sources  = $(filter %.c %.cpp %.S,$(1))
prepare_objects = $(addprefix $(BUILD_PATH)/,$(notdir $(addsuffix .o,$(call \
	filter_sources,$(call relpath,$(1))))))

INCLUDE_PATHS  := $(call relpath,$(SRC_PATH))
SOURCE_OBJECTS := $(call prepare_objects,$(call list_files,$(SRC_PATH)))

TEENSY3_CORE_PATH := $(realpath $(PROJECT_PATH)/teensy/src/teensy3)

# copy all of these files to the local TEENSY3_COPY_PATH folder
TEENSY3_FILES = $(filter-out $(TEENSY3_EXCLUDES),$(call \
	relpath2,$(TEENSY3_CORE_PATH),$(call list_files,$(TEENSY3_CORE_PATH))))

# delete these files from the local TEENSY3_COPY_PATH folder
TEENSY3_EXCLUDES = Makefile main.cpp

# add file in TEENSY3_COPY_PATH to compile list
INCLUDE_PATHS += $(TEENSY3_COPY_PATH)

CORE_LIB_OBJECTS = $(call prepare_objects,$(TEENSY3_FILES))

# all files in TEENSY3_CORE_PATH with global path
CORE_COPY_DEPENDENCIES =$(addprefix $(TEENSY3_CORE_PATH)/,$(TEENSY3_FILES))

# (Internal) target definitions ahead
# This one builds TEENSY3_COPY and recursively builds LIBTEENSY3
.ONESHELL:
$(TEENSY3_COPY_PATH): $(CORE_COPY_DEPENDENCIES)
	@mkdir -p $(BIN_PATH)
	@mkdir -p $(TEENSY3_COPY_PATH)
	@for f in $(TEENSY3_FILES);\
		do \
			if [ ! -e $(TEENSY3_COPY_PATH)/$$f ]; \
			then \
				mkdir -p `dirname $(TEENSY3_COPY_PATH)/$$f`; \
				cp -fp $(TEENSY3_CORE_PATH)/$$f $(TEENSY3_COPY_PATH)/$$f; \
			fi \
		done
	@for f in $(TEENSY3_EXCLUDES); \
		do \
			if [ -e $(TEENSY3_COPY_PATH)/$$f ]; then \
				rm $(TEENSY3_COPY_PATH)/$$f; \
			fi \
		done
	@touch $(TEENSY3_COPY_PATH)
	@BUILD_TEENSY3_LIB=1 $(MAKE) $(LIBTEENSY3)

# Only build LIBTEENSY3 if called recursively
ifeq ($(BUILD_TEENSY3_LIB),1)
$(LIBTEENSY3):  $(CORE_LIB_OBJECTS)
else
$(LIBTEENSY3): $(TEENSY3_COPY_PATH) $(CORE_LIB_OBJECTS)
	@echo "Linking $@ ..."
	@$(AR) rcs $@ $(CORE_LIB_OBJECTS)
endif

LDFLAGS += -T$(TEENSY3_COPY_PATH)/mk20dx256.ld

# Add include folders and set source and object lists
CPPFLAGS    += $(addprefix -I,$(INCLUDE_PATHS))
ALL_OBJECTS  = $(SOURCE_OBJECTS) $(CORE_LIB_OBJECTS)

# Links the .elf
$(BUILD_PATH)/$(M_PROJECT).elf: $(LIBTEENSY3) $(SOURCE_OBJECTS) Makefile
	@echo "Linking "$@" ..."
	@$(CXX) $(LDFLAGS) -o $@ $(SOURCE_OBJECTS) $(LIBTEENSY3) $(LIBS)

# Creates the .hex from the .elf
$(BIN_PATH)/%.hex: $(BUILD_PATH)/%.elf
	@echo "Preparing $@ ..."
	@$(SIZE) $<
	@$(OBJCOPY) -O ihex -R .eeprom $< $@

# Creates the .eep from the .elf
$(BIN_PATH)/%.eep: $(BUILD_PATH)/%.elf
	@echo "Preparing $@ ..."
	@$(SIZE) $<
	@$(OBJCOPY) -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load \
		--no-change-warnings --change-section-lma .eeprom=0 $< $@

#************************************************************************
# Dedicated rules
#************************************************************************
define CXX_RULE
$(BUILD_PATH)/%.cpp.o: $(1)/%.cpp Makefile
	@echo "Compiling $$< ..."
	@$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) -o $$@ $$<
endef

define CC_RULE
$(BUILD_PATH)/%.c.o: $(1)/%.c Makefile
	@echo "Compiling $$< ..."
	@$(CC) -c $(CPPFLAGS) $(CFLAGS) -o $$@ $$<
endef

define ASM_RULE
$(BUILD_PATH)/%.S.o: $(1)/%.S Makefile
	@echo "Compiling $$< ..."
	@$(CC) -c $(CPPFLAGS) $(ASM_FLAGS) $(CFLAGS) -o $$@ $$<
endef

# All compile rules
$(foreach tmp,$(INCLUDE_PATHS), \
	$(eval $(call CC_RULE,$(tmp))) \
	$(eval $(call CXX_RULE,$(tmp))) \
	$(eval $(call ASM_RULE,$(tmp))))

# Include auto generated dependencies, don't fail on error
-include $(ALL_OBJECTS:.o=.d)
