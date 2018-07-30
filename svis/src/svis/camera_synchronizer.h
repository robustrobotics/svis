// Copyright 2016 Massachusetts Institute of Technology

#pragma once

#include <map>
#include <deque>
#include <string>

#include "svis/strobe_packet.h"
#include "svis/camera_packet.h"
#include "svis/sync_state.h"

namespace svis {

class CameraSynchronizer {
 public:
  CameraSynchronizer(std::size_t max_buffer_size);
  void PushStrobePacket(const StrobePacket& strobe_packet);
  void PushCameraPacket(const CameraPacket& camera_packet);
  bool GetSynchronizedTime(const std::string& sensor_name,
                           const uint64_t& frame_count,
                           double* timestamp) const;
  bool Synchronize();
  bool Synchronized();

 private:
  bool BuffersFull() const;
  void ComputeStrobeOffsets(const SyncState& state, std::vector<int> *offsets) const;
  bool SyncStateExists(const std::string& sensor_name) const;
  SyncState& GetSyncState(const std::string& sensor_name);
  bool GetSyncState(const std::string& sensor_name, const SyncState* state) const;
  bool ComputeBestOffset(const SyncState& state,
                         const std::vector<int>& offsets,
                         int* best_offset) const;
  void ResetFrameOffsets();
  bool FrameOffsetsConsistent() const;
  bool TimeSynchronized() const;

  std::size_t max_buffer_size_;
  std::map<std::string, SyncState> sync_states_;
  std::deque<StrobePacket> strobe_buffer_;
};

}  // namespace svis
