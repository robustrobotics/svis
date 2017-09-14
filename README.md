# svis
This is a meta-project that contains all of the components that create a Simple Visual Inertial Synchronization approach.  The components are defined as follows.

- svis: The core algorithmic C++ library that matches camera strobe messages to camera images and gives them a common time based on the teensy clock.
- svis_ros: The ROS wrapper C++ library that handles the input and output from the core svis library.
- svis_teensy: The C and C++ code, dependencies, and utilities that build the teensy driver and allow it to be uploaded to the board.
- svis_cad: The solidworks CAD files for the svis mounting structure.
- svis_pcb: The Eagle PCB files for the svis circuit board.

### Setup
The rough outline for running svis is as follows.

- Build svis and svis_ros code.
- Build svis_teensy code and upload it to the svis board.
- Start the Pointgrey camera driver.
- Start the svis_ros nodelet.
