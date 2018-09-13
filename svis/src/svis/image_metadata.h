// Copyright 2016 Massachusetts Institute of Technology

#pragma once

#include <string>

namespace svis {

struct ImageMetadata {
  std::string sensor_name = "";
  uint64_t frame_counter = 0;
  uint64_t frame_timestamp = 0;  // [microseconds]
  uint64_t sensor_timestamp = 0;  // [microseconds]
  uint64_t exposure_time = 0;  // [microseconds]
};

}  // namespace svis_ros
