// Copyright 2016 Massachusetts Institute of Technology

#pragma once

#include "svis/camera_packet.h"
#include "svis/strobe_packet.h"

namespace svis {

struct CameraStrobePacket {
  CameraPacket camera;
  StrobePacket strobe;
};

}  // namespace svis_ros
