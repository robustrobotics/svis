// Copyright 2016 Massachusetts Institute of Technology

#pragma once

#include "svis/image.h"
#include "svis/camera_info.h"
#include "svis/image_metadata.h"

namespace svis {

class CameraPacket {
 public:
  ImageMetadata metadata;
  CameraInfo info;
  Image image;
};

}  // namespace svis
