// Copyright 2016 Massachusetts Institute of Technology

#include "svis/camera_synchronizer.h"

namespace svis {

void CameraSynchronizer::CameraSynchronizer()
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
  SyncState& state = GetSyncState(camera_packet.sensor_name);
  state.camera_buffer.push_back(camera_packet);

  if (camera_buffer_.size() > max_buffer_size_) {
    camera_buffer_.pop_front();
  }
}

int CameraSynchronizer::GetFrameOffset(std::string sensor_name) {
  SyncState& state = GetSyncState(camera_packet.sensor_name);
  return state.frame_offset;
}

double CameraSynchronizer::GetSynchronizedTime(std::string sensor_name, uint64_t frame_count) {

}

bool CameraSynchronizer::IsSynchronized(std::string sensor_name) {
  SyncState& state = GetSyncState(sensor_name);
  return state.synchronized;
}

bool CameraSynchronizer::Synchronize(std::string sensor_name) {
  SyncState& state = GetSyncState(sensor_name);
  if (state.synchronized) {
    return;
  } else {
    std::vector<int> offsets(max_buffer_size_, 0);
    
    // do we have a sufficient number of samples for all inputs?
    std::size_t num_strobes = strobe_buffer_.size();
    std::size_t num_cameras = state.camera_buffer.size();
    if (num_cameras >= max_buffer_size_ && num_strobes >= max_buffer_size_) {
      // for all strobes; find first camera image that came after strobe and compute offset
      // if offset is consistent across buffer, assign frame_offset to input
      for (std::size_t i = 0; i < num_strobes; ++i) {
        const StrobePacket& strobe = strobe_buffer_[i];
        // look for matching camera packet
        for (std::size_t j = 0; j < num_strobes; ++j) {
          const CameraPacket& camera = state.camera_buffer[j];
          if (camera.timestamp > strobe.timestamp_ros) {
            offsets[i] = camera.metadata.frame_counter - strobe.count_total;
            break;
          }
        }
      }
    }

    printf("offsets: ");
    for (std::size_t i = 0; i < max_buffer_size_; ++i) {
      printf("%i, %i\n", i, offsets[i]);
    }
  }
}

}  // namespace svis
