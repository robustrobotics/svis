# project name (used to name the compiled .hex file)
TARGET = hello_serial_monitor

# source files
C_FILES := $(wildcard *.c)
CPP_FILES := $(wildcard *.cpp)

# path to your arduino installation
PROJECT_PATH = /home/fla/svis
# $(info PROJECT_PATH: ${PROJECT_PATH})

# path location for the arm-none-eabi compiler
COMPILER_PATH = /home/fla/gcc-arm-none-eabi-4_9-2014q4/bin
# $(info COMPILER_PATH: ${COMPILER_PATH})

#************************************************************************
# Define processor
#************************************************************************

# set your MCU type here, or make command line `make MCU=MK20DX256`
MCU=MK20DX256

# make it lower case
LOWER_MCU := $(subst A,a,$(subst B,b,$(subst C,c,$(subst D,d,$(subst E,e,$(subst F,f,$(subst G,g,$(subst H,h,$(subst I,i,$(subst J,j,$(subst K,k,$(subst L,l,$(subst M,m,$(subst N,n,$(subst O,o,$(subst P,p,$(subst Q,q,$(subst R,r,$(subst S,s,$(subst T,t,$(subst U,u,$(subst V,v,$(subst W,w,$(subst X,x,$(subst Y,y,$(subst Z,z,$(MCU)))))))))))))))))))))))))))
MCU_LD = $(LOWER_MCU).ld

#************************************************************************
# Location of utilities within project
#************************************************************************

# path for Teensy Loader, teensy_post_compile and teensy_reboot (on Linux)
TOOLS_PATH = $(abspath $(PROJECT_PATH)/teensy/tools)
# $(info TOOLS_PATH: ${TOOLS_PATH})

# path for the MCU library
MCU_PATH = $(abspath $(PROJECT_PATH)/teensy/tools/$(MCU_LD))
# $(info MCU_PATH: ${MCU_PATH})

# path for arduino source and includes
ARDUINO_PATH = $(abspath $(PROJECT_PATH)/teensy/src/teensy3)
# $(info ARDUINO_PATH: ${ARDUINO_PATH})

# arduino source files
ARDUINO_C_FILES := $(wildcard $(ARDUINO_PATH)/*.c)
# $(info ARDUINO_C_FILES: ${ARDUINO_C_FILES})
C_FILES += $(ARDUINO_C_FILES)
# $(info C_FILES: ${C_FILES})

ARDUINO_CPP_FILES := $(wildcard $(ARDUINO_PATH)/*.cpp)
# $(info ARDUINO_CPP_FILES: ${ARDUINO_CPP_FILES})
CPP_FILES += $(ARDUINO_CPP_FILES)
# $(info CPP_FILES: ${CPP_FILES})

#************************************************************************
# Settings below this point usually do not need to be edited
#************************************************************************

# configurable options
OPTIONS = -DF_CPU=48000000 -DUSB_SERIAL -DLAYOUT_US_ENGLISH -DUSING_MAKEFILE

# options needed by many Arduino libraries to configure for Teensy 3.0
OPTIONS += -D__$(MCU)__ -DARDUINO=10613 -DTEENSYDUINO=132

# CPPFLAGS = compiler options for C and C++
CPPFLAGS = -Wall -g -Os -mcpu=cortex-m4 -mthumb -MMD $(OPTIONS) -I$(ARDUINO_PATH)

# compiler options for C++ only
CXXFLAGS = -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti

# compiler options for C only
CFLAGS =

# linker options
LDFLAGS = -Os -Wl,--gc-sections,--defsym=__rtc_localtime=0 --specs=nano.specs -mcpu=cortex-m4 -mthumb -T$(MCU_PATH)

# additional libraries to link
LIBS = -lm

# names for the compiler programs
CC = $(COMPILER_PATH)/arm-none-eabi-gcc
CXX = $(COMPILER_PATH)/arm-none-eabi-g++
OBJCOPY = $(COMPILER_PATH)/arm-none-eabi-objcopy
SIZE = $(COMPILER_PATH)/arm-none-eabi-size

#************************************************************************
# Objects definition
#************************************************************************
# define objects for compilation
OBJS := $(C_FILES:.c=.o) $(CPP_FILES:.cpp=.o)

#************************************************************************
# Make targets
#************************************************************************
all: $(TARGET).hex

$(TARGET).elf: $(OBJS) $(MCU_PATH)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

%.hex: %.elf
	$(SIZE) $<
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# compiler generated dependency info
-include $(OBJS:.o=.d)

upload:
	$(TOOLS_PATH)/teensy_loader_cli/teensy_loader_cli -mmcu=$(LOWER_MCU) -wv $(TARGET).hex

clean:
	rm -f *.o *.d $(TARGET).elf $(TARGET).hex
