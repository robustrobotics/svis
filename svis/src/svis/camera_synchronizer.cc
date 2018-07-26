// Copyright 2016 Massachusetts Institute of Technology

#include <vector>
#include <limits>
#include <cmath>

#include "svis/camera_synchronizer.h"

namespace svis {

CameraSynchronizer::CameraSynchronizer()
  : max_buffer_size_(50) {
  // nothing
}

void CameraSynchronizer::PushStrobePacket(const StrobePacket& strobe_packet) {
  strobe_buffer_.push_back(strobe_packet);

  if (strobe_buffer_.size() > max_buffer_size_) {
    strobe_buffer_.pop_front();
  }
}

void CameraSynchronizer::PushCameraPacket(const CameraPacket& camera_packet) {
  SyncState& state = GetSyncState(camera_packet.metadata.sensor_name);
  state.camera_buffer.push_back(camera_packet);

  if (state.camera_buffer.size() > max_buffer_size_) {
    state.camera_buffer.pop_front();
  }
}

bool CameraSynchronizer::GetSyncState(const std::string sensor_name, const SyncState* state) const {
  auto it = sync_states_.find(sensor_name);

  if (it != sync_states_.end()) {
    state = &it->second;
    return true;
  } 

  return false;
}

CameraSynchronizer::SyncState& CameraSynchronizer::GetSyncState(const std::string sensor_name) {
  auto pair = sync_states_.find(sensor_name);

  if (pair != sync_states_.end()) {
    return pair->second;
  }

  return sync_states_[sensor_name];
}

bool CameraSynchronizer::BuffersFull() const {
  std::size_t num_strobes = strobe_buffer_.size();
  if (num_strobes < max_buffer_size_) {
    printf("(svis) Not enough strobes\n");
    return false;
  }

  for (const auto& pair : sync_states_) {
    const std::string& sensor_name = pair.first;
    const SyncState& state = pair.second;
    std::size_t num_cameras = state.camera_buffer.size();
    if (num_cameras < max_buffer_size_) {
      printf("(svis) %s input has %lu of %lu\n", sensor_name.c_str(), num_cameras, max_buffer_size_);
      return false;
    }
  }

  return true;
}


// find first camera image that causes the magnitude of the time offset to increase and compute frame offset
void CameraSynchronizer::ComputeStrobeOffsets(const SyncState& state, std::vector<int> *offsets) const {
  for (std::size_t i = 0; i < max_buffer_size_; ++i) {
    const StrobePacket& strobe = strobe_buffer_[i];
    // look for min time offset
    int frame_offset = 0;
    int frame_offset_last = 0;
    double time_offset = std::numeric_limits<double>::infinity();
    double time_offset_last = std::numeric_limits<double>::infinity();
    for (std::size_t j = 0; j < max_buffer_size_; ++j) {
      const CameraPacket& camera = state.camera_buffer[j];

      // get time offset
      time_offset_last = time_offset;
      time_offset = std::abs(camera.timestamp - strobe.timestamp_ros);

      // get frame offset
      frame_offset_last = frame_offset;
      frame_offset = camera.metadata.frame_counter - strobe.count_total;

      // assign last offset if the magnitude of our time offset increased?
      if (time_offset > time_offset_last) {
        // printf("frame_count: %lu, strobe_count: %i", camera.metadata.frame_counter, strobe.count_total);
        offsets->at(i) = frame_offset_last;
        break;
      }

      // handle last sample
      if ((j == max_buffer_size_ - 1)) {
        offsets->at(i) = frame_offset;
      }
    }
  }

  printf("offsets:\n");
  for (std::size_t i = 0; i < max_buffer_size_; ++i) {
    printf("%lu, %i\n", i, offsets->at(i));
  }
}

bool CameraSynchronizer::ComputeBestOffset(const SyncState& state,
                                           const std::vector<int>& offsets,
                                           int* best_offset) const {
  // bin offsets
  std::map<int, std::size_t> binned_offsets;
  for (const auto& offset : offsets) {
    binned_offsets[offset]++;
  }
  printf("bin count: %lu\n", binned_offsets.size());

  // find most frequent bin
  int temp_best_offset = 0;
  std::size_t temp_best_offset_count = 0;
  for (const auto& pair : binned_offsets) {
    const auto& offset = pair.first;
    const auto& offset_count = pair.second;
    if (offset_count > temp_best_offset_count) {
      temp_best_offset = offset;
      temp_best_offset_count = offset_count;
    }
  }

  // make sure most frequent bin is clost to buffer size
  if (temp_best_offset_count > (max_buffer_size_ - 2)) {
    *best_offset = temp_best_offset;
    printf("best_offset: %i\n", *best_offset);
    return true;
  }
  
  return false;
}

bool CameraSynchronizer::Synchronized() {
  for (auto& pair : sync_states_) {
    SyncState& state = pair.second;
    if (!state.synchronized) {
      return false;
    }
  }

  return true;
}

bool CameraSynchronizer::Synchronize() {
  if (Synchronized()) {
    return true;
  }

  // do we have a sufficient number of strobe samples?
  if (!BuffersFull()) {
    return false;
  }
  printf("(svis) Have enough samples for synchronization\n");

  // compute offsets for strobe samples for all inputs
  for (auto& pair : sync_states_) {
    const std::string& sensor_name = pair.first;
    SyncState& state = pair.second;
    printf("synchronizing %s input\n", sensor_name.c_str());
    
    // get offsets
    std::vector<int> offsets(max_buffer_size_, 0);
    ComputeStrobeOffsets(state, &offsets);

    // find best offset
    int best_offset = 0;
    if (ComputeBestOffset(state, offsets, &best_offset)) {
      // TODO(jakeware): additional verification step here?
      state.frame_offset = best_offset;
      state.synchronized = true;
    } else {
      return false;
    }
  }

  return true;
}

}  // namespace svis
