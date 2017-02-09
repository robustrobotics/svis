set(TEENSY_FREQUENCY "96" CACHE STRING "Frequency of the Teensy MCU (Mhz)" FORCE)
set(TEENSY_USB_MODE "SERIAL" CACHE STRING "What kind of USB device the Teensy should emulate" FORCE)

import_arduino_library(Wire)
import_arduino_library(Adafruit_Sensor)
import_arduino_library(Adafruit_LSM303DLHC)
import_arduino_library(Adafruit_L3GD20_U)
import_arduino_library(Adafruit_9DOF)

add_teensy_executable(adafruit_9dof src/adafruit_9dof.cpp)
