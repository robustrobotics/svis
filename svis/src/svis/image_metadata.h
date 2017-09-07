// Copyright 2016 Massachusetts Institute of Technology

#pragma once

namespace svis {

struct ImageMetadata {
  uint32_t timestamp = 0;
  uint32_t gain = 0;
  uint32_t shutter = 0;
  uint32_t brightness = 0;
  uint32_t exposure = 0;
  uint32_t white_balance = 0;
  uint32_t frame_counter = 0;
  uint32_t strobe_pattern = 0;
  uint32_t gpio_state = 0;
  uint32_t roi_position = 0;
};

}  // namespace svis_ros
