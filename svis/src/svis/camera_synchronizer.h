// Copyright 2016 Massachusetts Institute of Technology

#pragma once

#include <map>
#include <deque>
#include <string>

#include "svis/strobe_packet.h"
#include "svis/camera_packet.h"
#include "svis/camera_strobe_packet.h"

namespace svis {

class CameraSynchronizer {
 public:
  CameraSynchronizer();
  void PushStrobePacket(const StrobePacket& strobe_packet);
  void PushCameraPacket(const CameraPacket& camera_packet);
  int GetFrameOffset(std::string sensor_name);
  double GetSynchronizedTime(std::string sensor_name, uint64_t frame_count);
  bool IsSynchronized(std::string sensor_name);
  bool Synchronize(std::string sensor_name);
  // void CheckSynchronization();
  
 private:
  struct SyncState {
    bool synchronized = false;
    int frame_offset = 0;
    std::deque<CameraPacket> camera_buffer;
  };

  SyncState& GetSyncState(std::string sensor_name) { return sync_states_[sensor_name]; }

  int max_buffer_size_;
  std::map<std::string, SyncState> sync_states_;
  std::deque<StrobePacket> strobe_buffer_;
};

}  // namespace svis
