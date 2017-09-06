// Copyright 2016 Massachusetts Institute of Technology

#pragma once

#include <string>
#include <vector>

#include "svis/header.h"

namespace svis {

struct Image {
  Header header;
  uint32_t height = 0;
  uint32_t width = 0;
  std::string encoding = "";
  uint8_t is_bigendian = 0;
  uint32_t step = 0;
  std::vector<uint8_t> data;
}

}  // namespace svis
