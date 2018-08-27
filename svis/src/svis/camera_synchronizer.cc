// Copyright 2016 Massachusetts Institute of Technology

#include <vector>
#include <limits>
#include <cmath>

#include "svis/camera_synchronizer.h"

namespace svis {

CameraSynchronizer::CameraSynchronizer(std::size_t max_buffer_size)
  : max_buffer_size_(max_buffer_size) {
  // nothing
}

void CameraSynchronizer::PushStrobePacket(const StrobePacket& strobe_packet) {
  strobe_buffer_.push_back(strobe_packet);

  if (strobe_buffer_.size() > max_buffer_size_) {
    strobe_buffer_.pop_front();
  }
}

void CameraSynchronizer::PushCameraPacket(const CameraPacket& camera_packet) {
  SyncState& state = sync_states_[camera_packet.metadata.sensor_name];
  state.camera_buffer_.push_back(camera_packet);
  printf("[CameraSynchronizer::PushCameraPacket] camera_buffer size: %lu\n", state.camera_buffer_.size());

  if (state.camera_buffer_.size() > max_buffer_size_) {
    state.camera_buffer_.pop_front();
  }
}

bool CameraSynchronizer::GetSynchronizedTime(const std::string& sensor_name,
                                             const uint64_t& frame_count,
                                             double* timestamp) const {
  if (!SyncStateExists(sensor_name)) {
    return false;
  }

  const SyncState& state = sync_states_.at(sensor_name);
  printf("[CameraSynchronizer::GetSynchronizedTime] Searching for frame: %lu\n", frame_count);
  printf("[CameraSynchronizer::GetSynchronizedTime] Camera buffer size: %lu\n", state.camera_buffer_.size());

  // find matching camera packet
  const CameraPacket* matched_camera = nullptr;
  for (const auto& camera : state.camera_buffer_) {
    // printf("Checking %lu\n", camera.metadata.frame_counter);
    if (camera.metadata.frame_counter == frame_count) {
      printf("[CameraSynchronizer::GetSynchronizedTime] Matched frame %lu\n", frame_count);
      matched_camera = &camera;
    }
  }

  // bail if we failed to match the camera packet
  if (!matched_camera) {
    printf("[CameraSynchronizer::GetSynchronizedTime] Failed to match camera\n");
    return false;
  }

  // find matching strobe packet
  printf("[CameraSynchronizer::GetSynchronizedTime] Searching for matching strobe\n");
  for (const auto& strobe : strobe_buffer_) {
    // printf("frame_count: %lu, strobe.count_total: %i, state.frame_offset: %i\n",
           // frame_count,
           // strobe.count_total,
           // state.frame_offset_);
    if (frame_count == (strobe.count_total + state.frame_offset_)) {
      printf("[CameraSynchronizer::GetSynchronizedTime] Found matching strobe: %i\n", strobe.count_total);
      // double sensor_timestamp = static_cast<double>(matched_camera->metadata.sensor_timestamp);
      // double frame_timestamp = static_cast<double>(matched_camera->metadata.frame_timestamp);
      // double sensor_time_offset = (sensor_timestamp - frame_timestamp) / 1000000.0;  // seconds
      // printf("sensor_time_offset: %f\n", sensor_time_offset);
      double temp_timestamp = strobe.timestamp_ros;  // + sensor_time_offset;

      // TODO(jakeware): sanity check timestamp before assignment
      *timestamp = temp_timestamp;

      return true;
    }
  }

  return false;
}

bool CameraSynchronizer::SyncStateExists(const std::string& sensor_name) const {
  auto pair = sync_states_.find(sensor_name);

  if (pair != sync_states_.end()) {
    return true;
  } 

  return false;
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
    std::size_t num_cameras = state.camera_buffer_.size();
    if (num_cameras < max_buffer_size_) {
      printf("(svis) %s input has %lu of %lu\n", sensor_name.c_str(), num_cameras, max_buffer_size_);
      return false;
    }
  }

  return true;
}

// find first camera image that causes the magnitude of the time offset to increase and compute frame offset
void CameraSynchronizer::ComputeStrobeOffsets(const SyncState& state, std::vector<int> *offsets) const {
  std::size_t num_offsets = offsets->size();
  std::vector<double> time_offsets(num_offsets, 0.0);
  std::vector<std::size_t> camera_index(num_offsets, 0);
  for (std::size_t i = 0; i < num_offsets; ++i) {
    const StrobePacket& strobe = strobe_buffer_[i];
    // look for min time offset
    int frame_offset = 0;
    int frame_offset_last = 0;
    double time_offset = std::numeric_limits<double>::infinity();
    double time_offset_last = std::numeric_limits<double>::infinity();
    for (std::size_t j = 0; j < max_buffer_size_; ++j) {
      const CameraPacket& camera = state.camera_buffer_[j];

      // get time offset
      time_offset_last = time_offset;
      time_offset = std::abs(camera.timestamp - strobe.timestamp_ros);

      // get frame offset
      frame_offset_last = frame_offset;
      frame_offset = camera.metadata.frame_counter - strobe.count_total;

      // assign last offset if the magnitude of our time offset increased?
      if (time_offset > time_offset_last) {
        printf("frame_count: %lu, strobe_count: %i, time_offset: %f\n", camera.metadata.frame_counter, strobe.count_total, time_offset_last);
        offsets->at(i) = frame_offset_last;
	time_offsets[i] = time_offset_last;
	camera_index[i] = j - 1;
        break;
      }
    }
  }

  printf("[CameraSynchronizer::ComputeStrobeOffsets] offsets:\n");
  for (std::size_t i = 0; i < num_offsets; ++i) {
    printf("%lu, %i, %f, %lu\n", i, offsets->at(i), time_offsets[i], camera_index[i]);
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
  printf("[CameraSynchronizer::ComputeBestOffset] bin count: %lu\n", binned_offsets.size());

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

  // make sure most frequent bin is close to buffer size
  if (temp_best_offset_count > (offsets.size() - 2)) {
    *best_offset = temp_best_offset;
    printf("[CameraSynchronizer::ComputeBestOffset] best_offset: %i\n", *best_offset);
    return true;
  }
  
  return false;
}

bool CameraSynchronizer::Synchronized() {
  for (auto& pair : sync_states_) {
    SyncState& state = pair.second;
    if (!state.synchronized_) {
      printf("[CameraSynchronizer::Synchronized] %s not synchronized\n", pair.first.c_str());
      return false;
    }
  }

  return true;
}

bool CameraSynchronizer::FrameOffsetsConsistent() const {
  int reference_offset = sync_states_.begin()->second.frame_offset_;
  for (auto& pair : sync_states_) {
    const SyncState& state = pair.second;

    if (state.frame_offset_ != reference_offset) {
      printf("[CameraSynchronizer::FrameOffsetsConsistent] Offsets do not match after synchronization\n");
      return false;
    }
  }

  return true;
}

void CameraSynchronizer::ResetFrameOffsets() {
  printf("[CameraSynchronizer::ResetFrameOffsets]\n");
  for (auto& pair : sync_states_) {
    SyncState& state = pair.second;
    state.ResetFrameOffset();
  }
}

bool CameraSynchronizer::Synchronize() {
  // do we have a sufficient number of strobe samples?
  if (!BuffersFull()) {
    return false;
  }
  printf("[CameraSynchronizer::Synchronize] Have enough samples for synchronization\n");

  // compute offsets for strobe samples for all inputs
  for (auto& pair : sync_states_) {
    const std::string& sensor_name = pair.first;
    SyncState& state = pair.second;
    printf("[CameraSynchronizer::Synchronize] synchronizing %s input\n", sensor_name.c_str());
    
    // get offsets
    std::vector<int> offsets(max_buffer_size_ - 2, 0);
    ComputeStrobeOffsets(state, &offsets);

    // find best offset
    int best_offset = 0;
    if (ComputeBestOffset(state, offsets, &best_offset)) {
      state.SetFrameOffset(best_offset);
    } else {
      exit(1);
      return false;
    }
  }

  // check if frame offsets are consistent and reset if necessary
  if (!FrameOffsetsConsistent()) {
    ResetFrameOffsets();
    exit(1);
    return false;
  }

  // check if frame offsets lead to reasonable time associations
  // if (!TimeSynchronized()) {
  //   return false;
  // }

  // TODO(jakeware): Compute timeoffsets based on synchronization

  return true;
}

bool CameraSynchronizer::TimeSynchronized() const {
  // TODO(jakeware) Fill this in
  return true;
}

}  // namespace svis
