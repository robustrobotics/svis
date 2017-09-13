# svis_teensy
This package contains a portion of a Simple Visual Inertial Synchronization
approach that runs on a Teensy3.2 microcontroller board.  This includes the
relevant examples, utlities, documentation, and implementation source code.

### Requires:
- cmake
- gcc-arm-none-eabi-4_8-2014q3

### Setup:
This package need the ARM gcc binaries in the system path in order to build.

- Follow the instructions on [the Teensyduino install page](https://www.pjrc.com/teensy/td_download.html) to install both Arduino and the Teensyduino add-on.
- Make sure to update permissions on the installer gui with `sudo chmod +x TeensyduinoInstall.linux64`.
- Copy `arduino-X.Y.Z/hardware/tools/arm` to `/path/to/gcc-arm-none-eabi-4_8-2014q3`
- Source the gcc folder with `export PATH=${PATH}:/path/to/gcc-arm-none-eabi-4_8-2014q3`.

### Build:
- `cd /path/to/svis_teensy`
- `mkdir build`
- `cd build`
- `cmake ..`
- `make`

### Arduino:
This project does not use teensyduino, the software designed by pjrc.com to work
with the teensy boards.  Instead, the source and dependencies will be built with
cmake.  The utilities such as `teensy_loader_cli` and `usb_raw_hid_test` are
still built using make.

#### Install UDev Rules:
Ubuntu and other modern Linux distibutions use udev to manage device files when
USB devices are added and removed. By default, udev will create a device with
read-only permission which will not allow to you download code. You must place
the udev rules file at `/etc/udev/rules.d/49-teensy.rules`.

```
sudo cp 49-teensy.rules /etc/udev/rules.d/
```

### Programming:
This project uses the `teensy_loader_cli` instead of the Arduino IDE to program
the teensy microcontroller over USB.  First, build the `teensy_loader_cli` by
following the README in `svis_teensy/utilities/teensy_loader_cli`.  Then execute
the following command to upload your binary once you have run the top level
cmake build to generate the .hex binary file.

```
teensy_loader_cli -mmcu=mk20dx256 -wv svis_teensy/build/bin/file_name.elf.hex
```

Press the button on the teensy when requested to put the device into HalfKay
bootloader mode and finish uploading the program.
