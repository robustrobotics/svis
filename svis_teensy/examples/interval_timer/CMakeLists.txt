set(TEENSY_FREQUENCY "96" CACHE STRING "Frequency of the Teensy MCU (Mhz)" FORCE)
set(TEENSY_USB_MODE "SERIAL" CACHE STRING "What kind of USB device the Teensy should emulate" FORCE)

add_teensy_executable(interval_timer src/interval_timer.cpp)
