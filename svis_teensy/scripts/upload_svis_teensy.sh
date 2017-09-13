#!/bin/sh
echo "Uploading svis_teensy with teensy_loader_cli."
echo "Press program button when prompted."
../utilities/teensy_loader_cli/teensy_loader_cli -mmcu=mk20dx256 -wv ../build/bin/svis_teensy.elf.hex
