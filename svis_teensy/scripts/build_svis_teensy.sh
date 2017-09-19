#!/bin/sh
echo "Downloading arm-none-eabi compiler binaries."
wget https://launchpad.net/gcc-arm-embedded/4.8/4.8-2014-q3-update/+download/gcc-arm-none-eabi-4_8-2014q3-20140805-linux.tar.bz2 -P ~/

echo "Extracting arm-none-eabi compiler."
mkdir -p ~/gcc-arm-none-eabi-4_8-2014q3
tar -xf ~/gcc-arm-none-eabi-4_8-2014q3-20140805-linux.tar.bz2 -C ~/

echo "Copying CMSIS DSP library."
cp ../dependencies/libarm_cortexM4l_math.a ~/gcc-arm-none-eabi-4_8-2014q3/arm-none-eabi/lib/

echo "Adding arm-none-eabi compiler to path."
export PATH=${HOME}/gcc-arm-none-eabi-4_8-2014q3/bin:$PATH

echo "Building svis_teensy"
(mkdir -p ../build && cd ../build && cmake .. && make)

echo "Building teensy_loader_cli."
(cd ../utilities/teensy_loader_cli &&  make)
