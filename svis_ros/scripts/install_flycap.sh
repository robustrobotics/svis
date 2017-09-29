#!/bin/sh

echo "Installing flycapture apt dependencies"
sudo apt install libraw1394-11 libavcodec-ffmpeg56 libavformat-ffmpeg56 libswscale-ffmpeg3 libswresample-ffmpeg1 libavutil-ffmpeg54 libgtkmm-2.4-dev libglademm-2.4-dev libgtkglextmm-x11-1.2-dev libusb-1.0-0

echo "Downloading flycapture SDK tar file"
wget https://www.dropbox.com/s/41w2mzdpnmx949g/flycapture2-2.11.3.164-amd64-pkg.tgz

echo "Extracting flycapture SDK tar file"
tar -xvf flycapture2-2.11.3.164-amd64-pkg.tgz

echo "Running flycapture install scipt"
cd ./flycapture2-2.11.3.164-amd64
sudo sh install_flycapture.sh
