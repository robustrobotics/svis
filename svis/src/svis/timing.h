// Copyright 2016 Massachusetts Institute of Technology

#pragma once

#include <limits>

namespace svis {

// [seconds]
struct Timing {
  float rawhid_recv = std::numeric_limits<float>::quiet_NaN();
  float ros_spin_once = std::numeric_limits<float>::quiet_NaN();
  float check_checksum = std::numeric_limits<float>::quiet_NaN();
  float parse_header = std::numeric_limits<float>::quiet_NaN();
  float parse_imu = std::numeric_limits<float>::quiet_NaN();
  float parse_strobe = std::numeric_limits<float>::quiet_NaN();
  float compute_strobe_total = std::numeric_limits<float>::quiet_NaN();
  float publish_imu_raw = std::numeric_limits<float>::quiet_NaN();
  float publish_strobe_raw = std::numeric_limits<float>::quiet_NaN();
  float push_imu = std::numeric_limits<float>::quiet_NaN();
  float push_strobe = std::numeric_limits<float>::quiet_NaN();
  float compute_offsets = std::numeric_limits<float>::quiet_NaN();
  float filter_imu = std::numeric_limits<float>::quiet_NaN();
  float publish_imu = std::numeric_limits<float>::quiet_NaN();
  float associate = std::numeric_limits<float>::quiet_NaN();
  float publish_camera = std::numeric_limits<float>::quiet_NaN();
  float update = std::numeric_limits<float>::quiet_NaN();
  float period = std::numeric_limits<float>::quiet_NaN();
};

}  // namespace svis_ros
