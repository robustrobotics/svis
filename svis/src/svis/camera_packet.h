// Copyright 2016 Massachusetts Institute of Technology

#pragma once

#include "svis/image_metadata.h"

namespace svis {

struct CameraPacket {
  double timestamp;
  ImageMetadata metadata;
};

}  // namespace svis
