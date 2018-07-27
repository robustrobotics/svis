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
project(svis)
set(${PROJECT_NAME}_VERSION 0.0.0)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# Options
option(BUILD_DOCUMENTATION "Use Doxygen to create the HTML based API documentation" OFF)
option(BUILD_TESTS "Use GTest to build and test libraries" OFF)

# Definitions
# add_definitions("-std=c++11")  # can't do this because of C build
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif (NOT CMAKE_BUILD_TYPE)

# CMake Includes
include(cmake/setup.cmake)
include(cmake/install.cmake)

#-------------------------------------------------------------------------------
# Dependencies
#-------------------------------------------------------------------------------
find_package(libusb REQUIRED)

#-------------------------------------------------------------------------------
# Include Directories
#-------------------------------------------------------------------------------
include_directories(SYSTEM
  src
  )

#-------------------------------------------------------------------------------
# Configuration
#-------------------------------------------------------------------------------
# Pkg_config Configuration
install_pkg_config_file(${PROJECT_NAME}
  DESCRIPTION "A simple visual inertial synchronization approach."
  LIBS ${PROJECT_NAME}
  VERSION ${${PROJECT_NAME}_VERSION}
  REQUIRES #Eigen3
  CFLAGS "-std=c++11")

# CMake Configuration
# NOTE: You need to edit the config file (${PROJECT_NAME}Config.cmake.in) manually!
configure_file(${PROJECT_SOURCE_DIR}/cmake/templates/${PROJECT_NAME}Config.cmake.in
  "${PROJECT_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Config.cmake" @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/cmake/templates/${PROJECT_NAME}ConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY)
install(FILES 
  ${PROJECT_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Config.cmake 
  ${PROJECT_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake 
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/${PROJECT_NAME})

#-------------------------------------------------------------------------------
# Library: svis
#-------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/svis/camera_packet.h
  src/svis/camera_strobe_packet.h
  src/svis/camera_synchronizer.cc
  src/svis/camera_synchronizer.h
  src/svis/header.h
  src/svis/header_packet.h
  src/svis/image_metadata.h
  src/svis/imu_packet.h
  src/svis/strobe_packet.h
  src/svis/svis.cc
  src/svis/svis.h
  src/svis/sync_state.h
  src/svis/timing.h
  )

target_link_libraries(${PROJECT_NAME}
  svis_hid
  )

set_target_properties(${PROJECT_NAME} PROPERTIES 
    COMPILE_FLAGS "-std=c++11 -Wall -fPIC")

# Install
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
install(DIRECTORY src/${PROJECT_NAME} DESTINATION include
  FILES_MATCHING PATTERN "*.h")

#-------------------------------------------------------------------------------
# Library: svis_hid
#-------------------------------------------------------------------------------
add_library(svis_hid SHARED
  src/svis_hid/svis_hid.c
  src/svis_hid/svis_hid.h
  )

target_link_libraries(svis_hid
  ${LibUSB_LIBRARIES}
  )

set_target_properties(svis_hid PROPERTIES
    COMPILE_FLAGS "-Wall -fPIC")

# Install
install(TARGETS svis_hid
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
install(DIRECTORY src/svis_hid DESTINATION include
  FILES_MATCHING PATTERN "*.h")

#-------------------------------------------------------------------------------
# Unit Tests
#-------------------------------------------------------------------------------
# If the BUILD_TESTS option is set, a Google test binary will be compiled.
if (BUILD_TESTS)
  enable_testing()
  
  set(TEST_SOURCES
    test/test.cc)

  add_executable(${PROJECT_NAME}_test ${TEST_SOURCES} ./test/gtest/gtest-all.cc)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${OpenCV_LIBS} ${CGAL_LIBRARIES} pthread)
  set_target_properties(${PROJECT_NAME}_test PROPERTIES COMPILE_FLAGS "-std=c++11 -Wno-deprecated-register")

  add_test(NAME ${PROJECT_NAME}_test COMMAND ${PROJECT_NAME}_test --gtest_color=yes)

  # Copy test images.
  file(COPY ./test/test_images DESTINATION ${CMAKE_BINARY_DIR})
endif()

#-------------------------------------------------------------------------------
# Documentation
#-------------------------------------------------------------------------------
if (BUILD_DOCUMENTATION)
  find_package(Doxygen)

  if (NOT DOXYGEN_FOUND)
    message(FATAL_ERROR
      "Doxygen is needed to build the documentation.")
  endif()

  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in
    ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile @ONLY)
  add_custom_target(doc ALL
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/doc
    COMMENT "Generatin API documentation with Doxygen" VERBATIM
    )
  install(DIRECTORY ${CMAKE_BINARY_DIR}/doc/html DESTINATION doc/${PROJECT_NAME})
  install(DIRECTORY ${CMAKE_BINARY_DIR}/doc/latex DESTINATION doc/${PROJECT_NAME})
endif()
