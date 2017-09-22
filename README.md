# svis
This is a meta-project that contains all of the components that create a Simple Visual Inertial Synchronization approach.  The components are defined as follows.

- svis: The core algorithmic C++ library that matches camera strobe messages to camera images and gives them a common time based on the teensy clock.
- svis_ros: The ROS wrapper C++ library that handles the input and output from the core svis library.
- svis_teensy: The C and C++ code, dependencies, and utilities that build the teensy driver and allow it to be uploaded to the board.
- svis_cad: The solidworks CAD files for the svis mounting structure.
- svis_pcb: The Eagle PCB files for the svis circuit board.

### Dependencies
A list of noteable dependencies can be found below.
- catkin_tools
- Ubuntu 16.04
- ROS Kinetic
- ros-kinetic-pointgrey-camera-driver
- libusb-dev

Use the following commands to install the required apt dependencies.
```
sudo apt install python-catkin-tools
sudo apt install ros-kinetic-pointgrey-camera-driver
sudo apt install libusb-dev
```
### Quick Start
The rough outline for running svis is as follows.  More details can be found in the individual package README files.
- Build svis and svis_ros code.
```
mkdir -p /path/to/catkin_ws/src
cd /path/to/catkin_ws/src
git clone http://github.com/jakeware/svis
cd ../
source /opt/ros/kinetic/setup.bash
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```
- Build svis_teensy code.
```
cd ./path/to/svis/svis_teensy/scripts
./build_svis_teensy.sh
```
- Install the udev rules for the teensy.
```
sudo cp /path/to/svis/svis_teensy/utilities/49-teensy.rules /etc/udev/rules.d/
```
- Upload teensy driver.  Make sure the teensy's micro USB cable is connected.  When prompted, press the program button on the teensy.
```
cd ./path/to/svis/svis_teensy/scripts
./upload_svis_teensy.sh
```
- Start the Pointgrey camera driver.
```
cd /path/to/catkin_ws
source ./devel/setup.bash
roslaunch svis_ros flea3.launch
```
- Start the svis_ros nodelet.
```
cd /path/to/catkin_ws
source ./devel/setup.bash
roslaunch svis_ros svis_ros_nodelet.launch
```
