// Copyright 2016 Massachusetts Institute of Technology

#pragma once

namespace svis_ros {

struct HeaderPacket {
  double timestamp_ros_rx = {0};  // time message was received
  uint16_t send_count = 0;
  uint8_t imu_count = 0;
  uint8_t strobe_count = 0;
};

}  // namespace svis_ros
