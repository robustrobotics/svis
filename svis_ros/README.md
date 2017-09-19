# svis_ros
This package contains the ROS wrapper for the core library of a Simple Visual Inertial Synchronization
approach that accepts camera strobe messages from the Teensy microcontroller and synchronizes them with camera image messages.

### Requires:
- catkin tools
- roscpp
- nodelet
- image_transport
- std_msgs
- sensor_msgs
- message_generation
- dynamic_reconfigure
- svis

### Build:
This is ros project designed to be built with catkin tools.  This project can be built as follows from within a ros workspace.

```
mkdir -p /path/to/catkin_ws/src
cd /path/to/catkin_ws/src
git clone http://github.com/jakeware/svis
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

### Pointgrey:
Note that there is a specialized launch file that configures a flea3 camera to correctly interface with the svis software and hardware components.  It has currently only been tested on a Flea3 camera, and can be run as follows.

```
source ./devel/setup.bash
roslaunch svis_ros flea3.launch
```
