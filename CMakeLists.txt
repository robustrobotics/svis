cmake_minimum_required(VERSION 2.8.3)

# Configure CCache if available
find_program(CCACHE_FOUND ccache)
if (CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif (CCACHE_FOUND)

project(svis_ros)

set(CMAKE_VERBOSE_MAKEFILE FALSE)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

add_definitions("-std=c++11")
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif (NOT CMAKE_BUILD_TYPE)

## Find catkin macros and libraries
set(REQ_CATKIN_PKGS   
  roscpp
  nodelet
  image_transport
  std_msgs
  sensor_msgs
  message_generation
)
find_package(catkin REQUIRED COMPONENTS ${REQ_CATKIN_PKGS})

# find_package(libusb-1.0 REQUIRED)
find_package(libusb REQUIRED)

add_message_files (
  FILES
  SvisImu.msg
  SvisStrobe.msg
  )

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  svis_ros
  )

catkin_package(
 INCLUDE_DIRS src
 LIBRARIES #${PROJECT_NAME}
 CATKIN_DEPENDS ${REQ_CATKIN_PKGS}
 DEPENDS
)

include_directories(
  src
  src/hid
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}/include
  # ${LIBUSB_1_INCLUDE_DIRS}
  ${LibUSB_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/svis_ros_nodelet.cc
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  ${LIBUSB_1_LIBRARIES}
  svis_hid
  )

target_link_libraries(${PROJECT_NAME}  
  ${catkin_LIBRARIES}
  # ${LIBUSB_1_LIBRARIES}
  ${LibUSB_LIBRARIES}
  svis_hid
)

install(TARGETS svis_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(svis_hid
  src/hid/hid.c
  )

target_link_libraries(svis_hid
  # ${LIBUSB_1_LIBRARIES}
  ${LibUSB_LIBRARIES}
)

install(TARGETS svis_hid
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
