# Copyright (c) 2015, Pierre-Andre Saulais <pasaulais@free.fr>
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer. 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# setup paths
set(TOOLCHAIN_PREFIX "$ENV{HOME}/gcc-arm-none-eabi-4_8-2014q3/bin/arm-none-eabi-" CACHE PATH "Path to ARM toolchain")
set(TEENSY_CORES_ROOT "${CMAKE_SOURCE_DIR}/dependencies/cores" CACHE PATH "Path to the Teensy 'cores' repository")
set(TEENSY_ROOT "${TEENSY_CORES_ROOT}/teensy3" CACHE PATH "Path to 'teensy3' directory in 'cores'")
set(ARDUINO_LIB_ROOT "${CMAKE_SOURCE_DIR}/dependencies/libraries" CACHE PATH "Path to the Arduino library directory")

# configure options
set(ARDUINO_VERSION "10613" CACHE STRING "Version of the Arduino SDK")
set(TEENSYDUINO_VERSION "133" CACHE STRING "Version of the Teensyduino SDK")
set(TEENSY_MODEL "MK20DX256" CACHE STRING "Model of the Teensy MCU")
set(TEENSY_FREQUENCY "48" CACHE STRING "Frequency of the Teensy MCU (Mhz)")
set_property(CACHE TEENSY_FREQUENCY PROPERTY STRINGS 96 72 48 24 16 8 4 2)
set(TEENSY_USB_MODE "SERIAL" CACHE STRING "What kind of USB device the Teensy should emulate")
set_property(CACHE TEENSY_USB_MODE PROPERTY STRINGS SERIAL HID SERIAL_HID MIDI RAWHID FLIGHTSIM)

# setup system and cross-compiling
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_CROSSCOMPILING 1)

# arm compiler binaries
set(CMAKE_C_COMPILER "${TOOLCHAIN_PREFIX}gcc" CACHE PATH "gcc" FORCE)
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PREFIX}g++" CACHE PATH "g++" FORCE)
set(CMAKE_AR "${TOOLCHAIN_PREFIX}ar" CACHE PATH "archive" FORCE)
set(CMAKE_LINKER "${TOOLCHAIN_PREFIX}ld" CACHE PATH "linker" FORCE)
set(CMAKE_NM "${TOOLCHAIN_PREFIX}nm" CACHE PATH "nm" FORCE)
set(CMAKE_OBJCOPY "${TOOLCHAIN_PREFIX}objcopy" CACHE PATH "objcopy" FORCE)
set(CMAKE_OBJDUMP "${TOOLCHAIN_PREFIX}objdump" CACHE PATH "objdump" FORCE)
set(CMAKE_STRIP "${TOOLCHAIN_PREFIX}strip" CACHE PATH "strip" FORCE)
set(CMAKE_PRINT_SIZE "${TOOLCHAIN_PREFIX}size" CACHE PATH "size" FORCE)
set(CMAKE_RANLIB "${TOOLCHAIN_PREFIX}ranlib" CACHE PATH "ranlib" FORCE)

# include teensy core files
include_directories("${TEENSY_ROOT}")

# compile options
set(ARM_FLAGS "-mcpu=cortex-m4 -mthumb" CACHE STRING "arm flags")
set(OPTIMIZE_FALGS "-0s")
set(BASE_FLAGS "${ARM_FLAGS} ${OPTIMIZE_FLAGS} -c -Wall -ffunction-sections -fdata-sections -nostdlib -fsingle-precision-constant")
set(CMAKE_C_FLAGS_RELEASE "${BASE_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${BASE_FLAGS} -fno-exceptions -felide-constructors -std=gnu++0x -fno-rtti")

# definitions
add_definitions("-DARDUINO=${ARDUINO_VERSION}")
add_definitions("-DTEENSYDUINO=${TEENSYDUINO_VERSION}")
add_definitions("-D__${TEENSY_MODEL}__")
add_definitions(-DTEENSY_VERSION=3.2)
add_definitions(-DTEENSY_BOARD=TEENSY31)
add_definitions(-DLAYOUT_US_ENGLISH)
add_definitions(-DUSB_VID=null)
add_definitions(-DUSB_PID=null)
add_definitions(-MMD)

# link options
set(LINKER_FLAGS "-Os -Wl,--gc-sections,--relax,--defsym=__rtc_localtime=0 ${ARM_FLAGS} -fsingle-precision-constant -T${TEENSY_ROOT}/mk20dx256.ld")
link_libraries(m)
set(LINKER_LIBS "-larm_cortexM4l_math" )  # TODO(jakeware): use link_libraries to include these
set(CMAKE_SHARED_LINKER_FLAGS "${LINKER_FLAGS}")
set(CMAKE_MODULE_LINKER_FLAGS "${LINKER_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${LINKER_FLAGS}")

# avoid known bug in linux
# error: "arm-none-eabi-gcc: error: unrecognized command line option '-rdynamic'"
unset(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)
unset(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS)

# Do not pass flags like '-ffunction-sections -fdata-sections' to the linker.
# This causes undefined symbol errors when linking.
set(CMAKE_CXX_LINK_EXECUTABLE "<CMAKE_CXX_COMPILER> <CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES> ${LINKER_LIBS}")
