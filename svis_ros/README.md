# svis_ros
This package contains the ROS portion of a Simple Visual Inertial Synchronization
approach that accepts camera strobe messages from the Teensy microcontroller and synchronizes them with camera image messages.

### Requires:
- mingw32
- mingw32-binutils
- mingw32-runtime

### Setup:
This package need the ARM gcc binaries in the system path in order to build.

- Download Arduino version 1.6.13.
- Copy `arduino-1.6.13/hardware/tools/arm` to `~/gcc-arm-none-eabi-4_8-2014q3`
- Source this folder
