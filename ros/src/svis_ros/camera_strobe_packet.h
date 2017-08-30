// Copyright 2016 Massachusetts Institute of Technology

#pragma once

#include "svis_ros/camera_packet.h"
#include "svis_ros/strobe_packet.h"

namespace svis_ros {

struct CameraStrobePacket {
 public:
  CameraPacket camera;
  StrobePacket strobe;
};

}  // namespace svis_ros
