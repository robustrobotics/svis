// Copyright 2016 Massachusetts Institute of Technology

#pragma once

namespace svis_ros {

struct ImuPacket {
  double timestamp_ros_rx = 0.0;  // [seconds] time usb message was received in ros epoch
  double timestamp_ros = 0.0;  // [seconds] timestamp in ros epoch
  uint32_t timestamp_teensy_raw = 0;  // [microseconds] timestamp in teensy epoch
  double timestamp_teensy = 0.0;  // [seconds] timestamp in teensy epoch
  int16_t acc_raw[3] = {0};  // counts
  float acc[3] = {0};  // m/s^2
  int16_t gyro_raw[3] = {0};  // counts
  float gyro[3] = {0};  // rad/sec
};

}  // namespace svis_ros
