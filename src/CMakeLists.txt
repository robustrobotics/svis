set(TEENSY_FREQUENCY "96" CACHE STRING "Frequency of the Teensy MCU (Mhz)" FORCE)
set(TEENSY_USB_MODE "SERIAL" CACHE STRING "What kind of USB device the Teensy should emulate" FORCE)

import_arduino_library(Wire)
import_arduino_library(MPU6050)
import_arduino_library(I2Cdev)

add_teensy_executable(svis_teensy svis_teensy.cpp)
