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

file(GLOB TEENSY_C_CORE_FILES
  "${TEENSY_ROOT}/*.c"
  "${TEENSY_ROOT}/*.S"
  )

file(GLOB TEENSY_CXX_CORE_FILES
  "${TEENSY_ROOT}/*.cpp"
  )

macro(add_teensy_executable TARGET_NAME SOURCES)
    # Determine the target flags for this executable.
    set(USB_MODE_DEF)
    if(${TEENSY_USB_MODE} STREQUAL SERIAL)
        set(USB_MODE_DEF USB_SERIAL)
    elseif(${TEENSY_USB_MODE} STREQUAL HID)
        set(USB_MODE_DEF USB_HID)
    elseif(${TEENSY_USB_MODE} STREQUAL SERIAL_HID)
        set(USB_MODE_DEF USB_SERIAL_HID)
    elseif(${TEENSY_USB_MODE} STREQUAL MIDI)
        set(USB_MODE_DEF USB_MIDI)
    elseif(${TEENSY_USB_MODE} STREQUAL RAWHID)
        set(USB_MODE_DEF USB_RAWHID)
    elseif(${TEENSY_USB_MODE} STREQUAL FLIGHTSIM)
        set(USB_MODE_DEF USB_FLIGHTSIM)
    else()
        message(FATAL_ERROR "Invalid USB mode: ${TEENSY_USB_MODE}")
    endif()

    set(TARGET_FLAGS "-D${USB_MODE_DEF} -DF_CPU=${TEENSY_FREQUENCY}000000 ${TEENSY_FLAGS}")
    set(TARGET_C_FLAGS "${TARGET_FLAGS} ${TEENSY_C_FLAGS}")
    set(TARGET_CXX_FLAGS "${TARGET_FLAGS} ${TEENSY_CXX_FLAGS}")

    # Build the Teensy 'core' library.
    # Per-target because of preprocessor definitions.
    add_library(${TARGET_NAME}_TeensyCore
        ${TEENSY_C_CORE_FILES}
        ${TEENSY_CXX_CORE_FILES}
    )
    set_source_files_properties(${TEENSY_C_CORE_FILES}
        PROPERTIES COMPILE_FLAGS ${TARGET_C_FLAGS})
    set_source_files_properties(${TEENSY_CXX_CORE_FILES}
        PROPERTIES COMPILE_FLAGS ${TARGET_CXX_FLAGS})

    set(FINAL_SOURCES ${TEENSY_LIB_SOURCES} ${SOURCES})
    
    # Add the Arduino library directory to the include path if found.
    if(EXISTS ${ARDUINO_LIB_ROOT})
        include_directories(${ARDUINO_LIB_ROOT})
    endif(EXISTS ${ARDUINO_LIB_ROOT})
    
    # Build the ELF executable.
    add_executable(${TARGET_NAME} ${FINAL_SOURCES})
    set_source_files_properties(${FINAL_SOURCES}
        PROPERTIES COMPILE_FLAGS ${TARGET_CXX_FLAGS})
    target_link_libraries(${TARGET_NAME} ${TARGET_NAME}_TeensyCore)
    set_target_properties(${TARGET_NAME} PROPERTIES
        OUTPUT_NAME ${TARGET_NAME}
        SUFFIX ".elf"
    )
    set(TARGET_ELF "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET_NAME}.elf")
    
    # Generate the hex firmware files that can be flashed to the MCU.
    set(EEPROM_OPTS -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0)
    set(HEX_OPTS -O ihex -R .eeprom)
    add_custom_command(OUTPUT ${TARGET_ELF}.eep
                       COMMAND ${CMAKE_OBJCOPY} ${EEPROM_OPTS} ${TARGET_ELF} ${TARGET_ELF}.eep
                       DEPENDS ${TARGET_ELF})
    add_custom_command(OUTPUT ${TARGET_ELF}.hex
                       COMMAND ${CMAKE_OBJCOPY} ${HEX_OPTS} ${TARGET_ELF} ${TARGET_ELF}.hex
                       DEPENDS ${TARGET_ELF})
    add_custom_target(${TARGET_NAME}_Firmware ALL
                      DEPENDS ${TARGET_ELF}.eep ${TARGET_ELF}.hex)
    add_dependencies(${TARGET_NAME}_Firmware ${TARGET_NAME})
endmacro(add_teensy_executable) 

macro(import_arduino_library LIB_NAME)
    # Check if we can find the library.
    if(NOT EXISTS ${ARDUINO_LIB_ROOT})
        message(FATAL_ERROR "Could not find the Arduino library directory")
    endif(NOT EXISTS ${ARDUINO_LIB_ROOT})
    set(LIB_DIR "${ARDUINO_LIB_ROOT}/${LIB_NAME}")
    if(NOT EXISTS "${LIB_DIR}")
        message(FATAL_ERROR "Could not find the directory for library '${LIB_NAME}'")
    endif(NOT EXISTS "${LIB_DIR}")
    
    # Add it to the include path.
    include_directories("${LIB_DIR}")
    
    # Mark source files to be built along with the sketch code.
    file(GLOB SOURCES_CPP ABSOLUTE "${LIB_DIR}" "${LIB_DIR}/*.cpp")
    foreach(SOURCE_CPP ${SOURCES_CPP})
        set(TEENSY_LIB_SOURCES ${TEENSY_LIB_SOURCES} ${SOURCE_CPP})
    endforeach(SOURCE_CPP ${SOURCES_CPP})
    file(GLOB SOURCES_C ABSOLUTE "${LIB_DIR}" "${LIB_DIR}/*.c")
    foreach(SOURCE_C ${SOURCES_C})
        set(TEENSY_LIB_SOURCES ${TEENSY_LIB_SOURCES} ${SOURCE_C})
    endforeach(SOURCE_C ${SOURCES_C})
endmacro(import_arduino_library)
