// Copyright 2017 Massachusetts Institute of Technology

#include <cstring>

#include "svis/svis.h"

namespace svis {

SVIS::SVIS()
  : camera_synchronizer_(50) {
  // set circular buffer max lengths
  imu_buffer_.set_capacity(30);
}

double SVIS::GetTimeOffset() const {
  return time_offset_;
}

bool SVIS::GetSynchronizedTime(const std::string& sensor_name,
                         const uint64_t& frame_count,
                         double* timestamp) const {
  return camera_synchronizer_.GetSynchronizedTime(sensor_name, frame_count, timestamp);
}


void SVIS::PushCameraPacket(const svis::CameraPacket& camera_packet) {
  camera_synchronizer_.PushCameraPacket(camera_packet);
}

void SVIS::OpenHID() {
  // open rawhid port
  // C-based example is 16C0:0480:FFAB:0200
  // Arduino-based example is 16C0:0486:FFAB:0200
  int r = rawhid_open(1, 0x16C0, 0x0486, 0xFFAB, 0x0200);

  // check return
  if (r <= 0) {
    printf("(svis) No svis_teensy device found.\n");
    return;
  } else {
    printf("(svis) Found svis_teensy device\n");
  }
}

int SVIS::ReadHID(std::vector<char>* buf) {
  // check if any Raw HID packet has arrived
  tic();
  int num = rawhid_recv(0, buf->data(), buf->size(), 220);
  timing_.rawhid_recv = toc();

  // check byte count
  if (num < 0) {
    printf("(svis) Error: reading, device went offline\n");
    rawhid_close(0);
    exit(1);
  } else if (num == 0) {
    // if (!frame_init_flag_) {
    //   printf("(svis) 0 bytes received\n");
    // }
  } else if (num > 0) {
    // resize vector
    buf->resize(num);
  } else {
    printf("(svis) Bad return value from rawhid_recv\n");
  }

  return num;
}

void SVIS::Update() {
  std::chrono::time_point<std::chrono::high_resolution_clock>
    t_update_start_ = std::chrono::high_resolution_clock::now();
  
  // read and return if empty or bad
  std::vector<char> buf(64, 0);
  if (ReadHID(&buf) <= 0) {
    return;
  }

  double packet_timestamp_ros = TimeNow();

  // debug print
  if (print_buffer_) {
    PrintBuffer(buf);
  }

  // check checksums and return if they don't match
  if (CheckChecksum(buf)) {
    return;
  }

  // parse packets
  std::vector<ImuPacket> imu_packets;
  std::vector<StrobePacket> strobe_packets;
  double packet_timestamp_teensy = ParseBuffer(buf, &imu_packets, &strobe_packets);

  // compute timestamp offset
  if (time_init_flag_) {
    std::size_t time_offset_sample_count = time_offset_vec_.size();
    if (time_offset_sample_count < max_time_offset_samples_) {
      time_offset_vec_.push_back(packet_timestamp_ros - packet_timestamp_teensy);
      return;
    } else if (time_offset_sample_count == max_time_offset_samples_) {
      ComputeTimeOffset();
      time_init_flag_ = false;
      return;
    } else {
      printf("(svis) Unexpected time_offset_vec size\n");
    }
  }

  // handle imu
  for (const auto& imu : imu_packets) {
    PushImu(imu);
  }

  PublishImuRaw(imu_packets);

  // handle strobe
  for (const auto& strobe : strobe_packets) {
    camera_synchronizer_.PushStrobePacket(strobe);
  }

  PublishStrobeRaw(strobe_packets);

  // filter and publish imu
  std::vector<ImuPacket> imu_packets_filt;
  FilterImu(&imu_packets_filt);
  // DecimateImu(&imu_packets_filt);
  for (std::size_t i = 0; i < imu_packets_filt.size(); i++) {
    usleep(500);
    PublishImu(imu_packets_filt[i]);
  }

  // if we have enough samples, sync strobes and camera
  if (!camera_synchronizer_.Synchronized()) {
    camera_synchronizer_.Synchronize();
  }

  std::chrono::duration<double> update_duration = std::chrono::high_resolution_clock::now() - t_update_start_;
  timing_.update = update_duration.count();
  PublishTiming(timing_);
  timing_ = svis::Timing(); // clear timing
}

double SVIS::ParseBuffer(const std::vector<char>& buf, std::vector<ImuPacket>* imu_packets, std::vector<StrobePacket>* strobe_packets) {
  HeaderPacket header;
  ParseHeader(buf, &header);
  ParseImu(buf, header, imu_packets);
  ParseStrobe(buf, header, strobe_packets);
  ComputeStrobeTotal(strobe_packets);

  uint32_t teensy_timestamp;
  memcpy(&teensy_timestamp, &buf[timestamp_index], sizeof(teensy_timestamp));
  double teensy_timestamp_sec = static_cast<double>(teensy_timestamp / 1e6);
  
  return teensy_timestamp_sec;
}

void SVIS::SendSetup() {
  std::vector<char> buf(64, 0);

  // header
  buf[0] = 0xAB;
  buf[1] = 0;

  // camera rate
  buf[2] = static_cast<uint8_t>(camera_rate_);  // Hz

  // gyro range
  buf[3] = gyro_sens_;  // FS_SEL

  // accel range
  buf[4] = acc_sens_;  // AFS_SEL

  printf("(svis) Sending configuration packet\n");
  rawhid_send(0, buf.data(), buf.size(), 100);
}

bool SVIS::CheckChecksum(const std::vector<char>& buf) {
  tic();

  // calculate checksum
  uint16_t checksum_calc = 0;
  for (int i = 0; i < send_buffer_size - 2; i++) {
    checksum_calc += buf[i] & 0xFF;
  }

  // get packet chechsum
  uint16_t checksum_orig = 0;
  std::memcpy(&checksum_orig, &buf[checksum_index], sizeof(checksum_orig));
  bool ret = (checksum_calc != checksum_orig);

  // check match and return result
  if (ret) {
    printf("(svis) checksum error [%02X, %02X] [%02X, %02X]\n",
                 buf[checksum_index] & 0xFF, buf[checksum_index + 1] & 0xFF, checksum_calc, checksum_orig);
  }

  timing_.check_checksum = toc();

  return ret;
}

void SVIS::ComputeTimeOffset() {
  std::size_t offset_size = time_offset_vec_.size();
  const size_t skip_count = offset_size >> 1;
  for (std::size_t i = skip_count; i < offset_size; ++i) {
    // printf("(svis): time_offset[%lu]: %f\n", i, time_offset_vec_[i]);
    time_offset_ += time_offset_vec_[i] / (offset_size - skip_count);
  }
  printf("(svis) Time Offset: %f\n", time_offset_);
}

void SVIS::ParseHeader(const std::vector<char>& buf, HeaderPacket* header) {
  tic();

  int ind = 0;

  // ros time
  header->timestamp_ros_rx = TimeNow();

  // send_count
  memcpy(&header->send_count, &buf[ind], sizeof(header->send_count));
  // printf("(svis) send_count: [%i, %i]\n", ind, header->send_count);
  ind += sizeof(header->send_count);

  // imu_packet_count
  memcpy(&header->imu_count, &buf[ind], sizeof(header->imu_count));
  // printf("(svis) imu_packet_count: [%i, %i]\n", ind, header->imu_count);
  ind += sizeof(header->imu_count);

  // strobe_packet_count
  memcpy(&header->strobe_count, &buf[ind], sizeof(header->strobe_count));
  // printf("(svis) strobe_packet_count: [%i, %i]\n", ind, header->strobe_count);
  ind += sizeof(header->strobe_count);

  timing_.parse_header = toc();
}

void SVIS::ParseImu(const std::vector<char>& buf, const HeaderPacket& header, std::vector<ImuPacket>* imu_packets) {
  tic();

  for (int i = 0; i < header.imu_count; i++) {
    ImuPacket imu;
    int ind = imu_index[i];

    // ros receive time
    imu.timestamp_ros_rx = header.timestamp_ros_rx;

    // raw teensy timestamp
    memcpy(&imu.timestamp_teensy_raw, &buf[ind], sizeof(imu.timestamp_teensy_raw));
    // printf("(svis) imu.timestamp: [%i, %i]\n", ind, imu.timestamp);
    ind += sizeof(imu.timestamp_teensy_raw);

    // convert to seconds
    imu.timestamp_teensy = static_cast<double>(imu.timestamp_teensy_raw) / 1000000.0;

    // teensy time in ros epoch
    imu.timestamp_ros = imu.timestamp_teensy + GetTimeOffset();
    
    // accel
    memcpy(&imu.acc_raw[0], &buf[ind], sizeof(imu.acc_raw[0]));
    ind += sizeof(imu.acc_raw[0]);
    memcpy(&imu.acc_raw[1], &buf[ind], sizeof(imu.acc_raw[1]));
    ind += sizeof(imu.acc_raw[1]);
    memcpy(&imu.acc_raw[2], &buf[ind], sizeof(imu.acc_raw[2]));
    ind += sizeof(imu.acc_raw[2]);
    // printf("(svis) imu.acc_raw: [%i, %i, %i]\n", imu.acc_raw[0], imu.acc_raw[1], imu.acc_raw[2]);

    // convert accel
    imu.acc[0] = static_cast<float>(imu.acc_raw[0]) / acc_sens_arr_[acc_sens_] * g_;
    imu.acc[1] = static_cast<float>(imu.acc_raw[1]) / acc_sens_arr_[acc_sens_] * g_;
    imu.acc[2] = static_cast<float>(imu.acc_raw[2]) / acc_sens_arr_[acc_sens_] * g_;
    // printf("(svis) imu.acc: [%0.2f, %0.2f, %0.2f]\n", imu.acc[0], imu.acc[1], imu.acc[2]);

    // gyro
    memcpy(&imu.gyro_raw[0], &buf[ind], sizeof(imu.gyro_raw[0]));
    ind += sizeof(imu.gyro_raw[0]);
    memcpy(&imu.gyro_raw[1], &buf[ind], sizeof(imu.gyro_raw[1]));
    ind += sizeof(imu.gyro_raw[1]);
    memcpy(&imu.gyro_raw[2], &buf[ind], sizeof(imu.gyro_raw[2]));
    ind += sizeof(imu.gyro_raw[2]);
    // printf("(svis) imu.gyro_raw: [%i, %i, %i]\n", imu.gyro_raw[0], imu.gyro_raw[1], imu.gyro_raw[2]);

    // convert gyro
    imu.gyro[0] = static_cast<float>(imu.gyro_raw[0]) / gyro_sens_arr_[gyro_sens_] * rad_per_deg_;
    imu.gyro[1] = static_cast<float>(imu.gyro_raw[1]) / gyro_sens_arr_[gyro_sens_] * rad_per_deg_;
    imu.gyro[2] = static_cast<float>(imu.gyro_raw[2]) / gyro_sens_arr_[gyro_sens_] * rad_per_deg_;
    // printf("(svis) imu.gyro: [%0.2f, %0.2f, %0.2f]\n", imu.gyro[0], imu.gyro[1], imu.gyro[2]);

    // save packet
    imu_packets->push_back(imu);

    timing_.parse_imu = toc();
  }
}

void SVIS::ParseStrobe(const std::vector<char>& buf,
                     const HeaderPacket& header,
                     std::vector<StrobePacket>* strobe_packets) {
  tic();

  for (uint i = 0; i < header.strobe_count; i++) {
    StrobePacket strobe;
    int ind = strobe_index[i];

    // ros time
    strobe.timestamp_ros_rx = header.timestamp_ros_rx;

    // timestamp
    memcpy(&strobe.timestamp_teensy_raw, &buf[ind], sizeof(strobe.timestamp_teensy_raw));
    // printf("(svis) strobe.timestamp: [%i, %i]\n", ind, strobe.timestamp);
    ind += sizeof(strobe.timestamp_teensy_raw);

    // convert to seconds
    strobe.timestamp_teensy = static_cast<double>(strobe.timestamp_teensy_raw) / 1000000.0;

    // teensy time in ros epoch
    strobe.timestamp_ros = strobe.timestamp_teensy + GetTimeOffset();

    // count
    memcpy(&strobe.count, &buf[ind], sizeof(strobe.count));
    // printf("(svis) strobe.count: [%i, %i]\n", ind, strobe.count);
    ind += strobe.count;

    // save packet
    strobe_packets->push_back(strobe);
  }

  timing_.parse_strobe = toc();
}

void SVIS::PushImu(const ImuPacket& imu_packet) {
  tic();

  imu_buffer_.push_back(imu_packet);

  // warn if buffer is at max size
  if (imu_buffer_.size() == imu_buffer_.max_size()) {
    printf("(svis) imu buffer at max size\n");
  }

  timing_.push_imu = toc();
}

void SVIS::DecimateImu(std::vector<ImuPacket>* imu_packets_filt) {
  // create filter packets
  while (imu_buffer_.size() >= static_cast<std::size_t>(imu_filter_size_) && imu_filter_size_ > 0) {
    for (int i = 0; i < imu_filter_size_; i++) {
      if (i == 0) {
        imu_packets_filt->push_back(imu_buffer_.front());
      }
      imu_buffer_.pop_front();
    }
  }
}

void SVIS::FilterImu(std::vector<ImuPacket>* imu_packets_filt) {
  tic();

  // create filter packets
  while (imu_buffer_.size() >= static_cast<std::size_t>(imu_filter_size_) && imu_filter_size_ > 0) {
    double timestamp_diff_mean = 0.0;
    double acc_mean[3] = {0.0};
    double gyro_mean[3] = {0.0};

    // get local time offset for better precision
    double first_timestamp = imu_buffer_.front().timestamp_ros;
    
    for (int i = 0; i < imu_filter_size_; i++) {
      ImuPacket temp_packet = imu_buffer_.front();
      imu_buffer_.pop_front();

      timestamp_diff_mean += (temp_packet.timestamp_ros - first_timestamp) / static_cast<double>(imu_filter_size_);
      for (uint j = 0; j < 3; j++) {
        acc_mean[j] += temp_packet.acc[j] / static_cast<double>(imu_filter_size_);
        gyro_mean[j] += temp_packet.gyro[j] / static_cast<double>(imu_filter_size_);
      }
    }

    ImuPacket filter_packet;
    filter_packet.timestamp_ros = first_timestamp + timestamp_diff_mean;
    for (uint j = 0; j < 3; j++) {
      filter_packet.acc[j] = acc_mean[j];
      filter_packet.gyro[j] = gyro_mean[j];
    }

    // save packet
    imu_packets_filt->push_back(filter_packet);
  }

  timing_.filter_imu = toc();
}

void SVIS::PrintBuffer(const std::vector<char>& buf) {
  printf("(svis) buffer: ");
  for (uint i = 0; i < buf.size(); i++) {
    printf("%02X ", buf[i] & 255);
  }
  printf("\n");
}

void SVIS::ComputeStrobeTotal(std::vector<StrobePacket>* strobe_packets) {
  tic();

  for (auto& str : (*strobe_packets)) {
    // printf("strobe_count_total: %u\n", strobe_count_total_);

    // initialize variables on first iteration
    if (strobe_count_total_ == 0 && strobe_count_last_ == 0) {
      // printf("init strobe count\n");
      strobe_count_total_ = 1;
      strobe_count_last_ = str.count_total;
      str.count_total = strobe_count_total_;
      continue;
    }

    // get count difference between two most recent strobe messages
    uint8_t diff = 0;
    if (str.count > strobe_count_last_) {
      // no rollover
      diff = str.count - strobe_count_last_;
    } else if (str.count < strobe_count_last_) {
      // rollover
      diff = (strobe_count_last_ + str.count);

      // handle rollover
      if (diff == 255) {
        // printf("(svis) Handle rollover\n");
        diff = 1;
      }
    } else {
      // no change
      printf("(svis) no change in strobe count\n");
    }

    // check diff value
    if (diff > 1 && !std::isinf(strobe_count_last_)) {  //  && !frame_init_flag_
      printf("(svis) detected jump in strobe count\n");
      // printf("(svis) diff: %i, last: %i, count: %i\n",
      //              diff,
      //              strobe_count_last_,
      //              strobe_packets[i].count);
    } else if (diff < 1 && !std::isinf(strobe_count_last_)) {
      printf("(svis) detected lag in strobe count\n");
    }

    // update count
    strobe_count_total_ += diff;

    // set packet total
    str.count_total = strobe_count_total_;

    // update last value
    strobe_count_last_ = str.count;
  }

  timing_.compute_strobe_total = toc();
}

void SVIS::tic() {
  tic_ = std::chrono::high_resolution_clock::now();
}

double SVIS::toc() {
  std::chrono::duration<double> duration = std::chrono::high_resolution_clock::now() - tic_;

  return duration.count();
}

void SVIS::SetPublishStrobeRawHandler(std::function<void(const std::vector<StrobePacket>&)> handler) {
  PublishStrobeRaw = handler;
}

void SVIS::SetPublishImuRawHandler(std::function<void(const std::vector<ImuPacket>&)> handler) {
  PublishImuRaw = handler;
}

void SVIS::SetPublishImuHandler(std::function<void(const ImuPacket&)> handler) {
  PublishImu = handler;
}

void SVIS::SetPublishTimingHandler(std::function<void(const Timing&)> handler) {
  PublishTiming = handler;
}

void SVIS::SetTimeNowHandler(std::function<double()> handler) {
  TimeNow = handler;
}

}  // namespace svis
