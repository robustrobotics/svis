# svis_teensy
This package contains a portion of a Simple Visual Inertial Synchronization
approach that runs on a Teensy3.2 microcontroller board.  This includes the
relevant examples, utlities, documentation, and implementation source code.

### Requires:
- cmake
- gcc-arm-none-eabi-4_8-2014q3
- CMSIS DSP Library

### Build:
Although the teensy driver is a simple cmake project, a script has been 
provided to install the dependencies, modify the system path, and build the binary hex file.  Note that this script will install the arm gcc compiler binaries into your home directory and the cmake build will look for them in this default location.  If you wish to modify the installation location, you will have to also modify the teensy-arm.toolchain.cmake file in `/path/to/svis/svis_teensy/cmake`.

```
./path/to/svis/svis_teensy/scripts/build_svis_teensy.sh
```

#### Install UDev Rules:
Ubuntu and other modern Linux distibutions use udev to manage device files when
USB devices are added and removed. By default, udev will create a device with
read-only permission which will not allow to you download code. You must place
the udev rules file at `/etc/udev/rules.d/49-teensy.rules`.

```
sudo cp /path/to/svis/svis_teensy/utilities/49-teensy.rules /etc/udev/rules.d/
```

### Programming:
Once the driver has been built, it will need to be uploaded to the teensy
device.  Before proceeding, make sure the teensy is connected to the computer with a micro USB
cable.  The first step is to run the upload script to start the process.

```
./path/to/svis/svis_teensy/scripts/upload_svis_teensy.sh
```

Press the button on the teensy when requested to put the device bootloader mode and finish uploading the program.

### Teensyduino Compatibility:
Note that this project does not use teensyduino, the software designed by pjrc.com to work
with the teensy boards.  Instead, the source and dependencies will be built with
cmake.  The utilities such as `teensy_loader_cli` and `usb_raw_hid_test` are
still built using make.  It is very likely that the code would build in the Teensyduino/Arduino editor, but that capability is not monitored or maintained.
