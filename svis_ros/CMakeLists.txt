cmake_minimum_required(VERSION 2.8.3)

# Configure CCache if available
find_program(CCACHE_FOUND ccache)
if (CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif (CCACHE_FOUND)

#-------------------------------------------------------------------------------
# Project Initialization
#-------------------------------------------------------------------------------
project(svis_ros)

# Options
option(BUILD_DOCUMENTATION "Use Doxygen to create the HTML based API documentation" OFF)
option(BUILD_TESTS "Use GTest to build and test libraries" OFF)

# Definitions
add_definitions("-std=c++11")
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif (NOT CMAKE_BUILD_TYPE)

#-------------------------------------------------------------------------------
# Dependencies
#-------------------------------------------------------------------------------
# Catkin dependencies
set(REQ_CATKIN_PKGS   
  roscpp
  nodelet
  image_transport
  std_msgs
  sensor_msgs
  message_generation
  fla_utils
  )
find_package(catkin REQUIRED COMPONENTS ${REQ_CATKIN_PKGS})

# External dependencies
find_package(svis REQUIRED)

#-------------------------------------------------------------------------------
# ROS Build
#-------------------------------------------------------------------------------
add_message_files (
  FILES
  SvisImu.msg
  SvisStrobe.msg
  SvisTiming.msg
  )

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  svis_ros
  )

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES svis_ros
  CATKIN_DEPENDS ${REQ_CATKIN_PKGS}
  DEPENDS svis
  )

#-------------------------------------------------------------------------------
# Include Directories
#-------------------------------------------------------------------------------
include_directories(
  src
  ${svis_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

#-------------------------------------------------------------------------------
# Library: svis_ros_nodelet
#-------------------------------------------------------------------------------
add_library(svis_ros_nodelet SHARED
  src/svis_ros_nodelet.cc
  )

target_link_libraries(svis_ros_nodelet
  svis_ros
  ${catkin_LIBRARIES}
  )

install(TARGETS svis_ros_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#-------------------------------------------------------------------------------
# Library: svis_ros
#-------------------------------------------------------------------------------
add_library(svis_ros SHARED
  src/svis_ros/svis_ros.cc
  )

add_dependencies(svis_ros
  svis_ros_generate_messages_cpp
  )

target_link_libraries(svis_ros
  ${svis_LIBRARIES}
  ${catkin_LIBRARIES}
  )

install(TARGETS svis_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
