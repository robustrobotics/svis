// Copyright 2016 Massachusetts Institute of Technology

#pragma once

#include "svis/image_metadata.h"

namespace svis {

class CameraPacket {
 public:
  double timestamp;
  ImageMetadata metadata;
};

}  // namespace svis
