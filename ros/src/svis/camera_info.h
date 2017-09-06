// Copyright 2016 Massachusetts Institute of Technology

#pragma once

#include <string>

#include "svis/header.h"

namespace svis {

struct CameraInfo {
  Header header;
  uint32_t height = 0;
  uint32_t width = 0;
  std::string distortion_model = "";
  std::vector<double> D;
  std::array<double, 9> K;
  std::array<double, 9> R;
  std::array<double, 12> P;
  uint32_t binning_x = 0;
  uint32_t binning_y = 0;
  // RegionOfInterest roi;  // not included
};

}  // namespace svis
