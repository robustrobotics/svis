# The name of your project (used to name the compiled .hex file)
TARGET = main

# Path to your arduino installation
PROJECTPATH = /home/fla/svis
# $(info PROJECTPATH: ${PROJECTPATH})

# Path location for the arm-none-eabi compiler
COMPILERPATH = /home/fla/gcc-arm-none-eabi-4_9-2014q4/bin
# $(info COMPILERPATH: ${COMPILERPATH})

#************************************************************************
# Location of utilities within project
#************************************************************************

# Path for Teensy Loader, teensy_post_compile and teensy_reboot (on Linux)
TOOLSPATH = $(abspath $(PROJECTPATH)/teensy/tools)
# $(info TOOLSPATH: ${TOOLSPATH})

# Path for the mk20dx256 library
MCUPATH = $(abspath $(PROJECTPATH)/teensy/tools/mk20dx256.ld)
# $(info MCUPATH: ${MCUPATH})

#************************************************************************
# Settings below this point usually do not need to be edited
#************************************************************************

# configurable options
OPTIONS = -DF_CPU=48000000 -DUSB_SERIAL -DLAYOUT_US_ENGLISH -DUSING_MAKEFILE

# options needed by many Arduino libraries to configure for Teensy 3.0
OPTIONS += -D__$(MCU)__ -DARDUINO=10613 -DTEENSYDUINO=132

# CPPFLAGS = compiler options for C and C++
CPPFLAGS = -Wall -g -Os -mcpu=cortex-m4 -mthumb -MMD $(OPTIONS) -I.

# compiler options for C++ only
CXXFLAGS = -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti

# compiler options for C only
CFLAGS =

# linker options
LDFLAGS = -Os -Wl,--gc-sections,--defsym=__rtc_localtime=0 --specs=nano.specs -mcpu=cortex-m4 -mthumb -T$(MCUPATH)

# additional libraries to link
LIBS = -lm

# names for the compiler programs
CC = $(COMPILERPATH)/arm-none-eabi-gcc
CXX = $(COMPILERPATH)/arm-none-eabi-g++
OBJCOPY = $(COMPILERPATH)/arm-none-eabi-objcopy
SIZE = $(COMPILERPATH)/arm-none-eabi-size

# automatically create lists of the sources and objects
# TODO: this does not handle Arduino libraries yet...
C_FILES := $(wildcard *.c)
CPP_FILES := $(wildcard *.cpp)
OBJS := $(C_FILES:.c=.o) $(CPP_FILES:.cpp=.o)

# the actual makefile rules (all .o files built by GNU make's default implicit rules)

all: $(TARGET).hex

$(TARGET).elf: $(OBJS) $(MCUPATH)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

%.hex: %.elf
	$(SIZE) $<
	$(OBJCOPY) -O ihex -R .eeprom $< $@

upload:
	$(TOOLSPATH)/teensy_loader_cli/teensy_loader_cli -mmcu=mk20dx256 -wv $(TARGET).hex

# compiler generated dependency info
-include $(OBJS:.o=.d)

clean:
	rm -f *.o *.d $(TARGET).elf $(TARGET).hex
