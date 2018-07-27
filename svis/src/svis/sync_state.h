// Copyright 2016 Massachusetts Institute of Technology

#include <deque>

#include "svis/camera_packet.h"

#pragma once

namespace svis {

class SyncState {
 public:
  void SetFrameOffset(const int frame_offset) {
    frame_offset_ = frame_offset;
    synchronized_ = true;
  }

  void ResetFrameOffset() {
    frame_offset_ = 0;
    synchronized_ = false;
    camera_buffer_.clear();
  }

  bool synchronized_ = false;
  int frame_offset_ = 0;
  std::deque<CameraPacket> camera_buffer_;
};

}  // namespace svis
