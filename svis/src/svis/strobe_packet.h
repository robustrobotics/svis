// Copyright 2016 Massachusetts Institute of Technology

#pragma once

namespace svis {

struct StrobePacket {
  double timestamp_ros_rx = 0.0;  // [seconds] time usb message was received in ros epoch
  double timestamp_ros = 0.0;  // [seconds] timestamp in ros epoch
  uint32_t timestamp_teensy_raw = 0;  // [microseconds] timestamp in teensy epoch
  double timestamp_teensy = 0.0;  // [seconds] timestamp in teensy epoch
  uint8_t count = 0;  // number of camera images
  uint32_t count_total = 0;  // total number of camera messages thus far
};

}  // namespace svis_ros
