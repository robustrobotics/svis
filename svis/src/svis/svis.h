// Copyright 2016 Massachusetts Institute of Technology

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <sys/ioctl.h>

#include <chrono>
#include <boost/circular_buffer.hpp>

#include "svis/timing.h"
#include "svis/header_packet.h"
#include "svis/imu_packet.h"
#include "svis/strobe_packet.h"
#include "svis/camera_packet.h"
#include "svis/camera_strobe_packet.h"
#include "svis/camera_synchronizer.h"

extern "C" {
#include "svis_hid/svis_hid.h"
}

namespace svis {

class SVIS {
 public:
  SVIS();
  void Update();
  void OpenHID();
  int ReadHID(std::vector<char>* buf);
  void SendSetup();
  void tic();
  double toc();
  double GetTimeOffset() const;
  // bool IsSynchronized() const;
  void PushCameraPacket(const svis::CameraPacket& camera_packet);
  
  void SetPublishStrobeRawHandler(std::function<void(const std::vector<StrobePacket>&)> handler);
  void SetPublishImuRawHandler(std::function<void(const std::vector<ImuPacket>&)> handler);
  void SetPublishImuHandler(std::function<void(const ImuPacket&)> handler);
  void SetPublishTimingHandler(std::function<void(const Timing&)> handler);
  void SetTimeNowHandler(std::function<double()> handler);

  // params
  int camera_rate_ = 0;
  int gyro_sens_ = 0;  // gyro sensitivity selection [0,3]
  int acc_sens_ = 0;  // acc sensitivity selection [0,3]
  int imu_filter_size_ = 0;
  int offset_sample_count_ = 5;
  float offset_sample_time_ = 0.5;  // [s]

  // timing
  Timing timing_;
  std::chrono::time_point<std::chrono::high_resolution_clock> t_pulse_;
  std::chrono::time_point<std::chrono::high_resolution_clock> tic_;

 private:
  double ParseBuffer(const std::vector<char>& buf,
                      std::vector<ImuPacket>* imu_packets,
                      std::vector<StrobePacket>* strobe_packets);
  bool CheckChecksum(const std::vector<char>& buf);
  void ComputeTimeOffset();
  void ParseHeader(const std::vector<char>& buf,
                     HeaderPacket* header);
  void ParseImu(const std::vector<char>& buf,
              const HeaderPacket& header,
              std::vector<ImuPacket>* imu_packets);
  void ParseStrobe(const std::vector<char>& buf,
                 const HeaderPacket& header,
                 std::vector<StrobePacket>* strobe_packets);
  void PushImu(const std::vector<ImuPacket>& imu_packets);
  void FilterImu(boost::circular_buffer<ImuPacket>* imu_buffer,
                 std::vector<ImuPacket>* imu_packets_filt);
  void DecimateImu(boost::circular_buffer<ImuPacket>* imu_buffer,
                 std::vector<ImuPacket>* imu_packets_filt);
  void PrintBuffer(const std::vector<char>& buf);
  void ComputeStrobeTotal(std::vector<StrobePacket>* strobe_packets);

  // handlers
  std::function<void(const std::vector<svis::StrobePacket>&)> PublishStrobeRaw;
  std::function<void(const std::vector<svis::ImuPacket>&)> PublishImuRaw;
  std::function<void(const svis::ImuPacket&)> PublishImu;
  std::function<void(const Timing&)> PublishTiming;
  std::function<double()> TimeNow;

  // buffers
  boost::circular_buffer<ImuPacket> imu_buffer_;

  // constants
  const double g_ = 9.80665;
  const double rad_per_deg_ = 0.0174533;

  // imu
  double gyro_sens_arr_[4] = {131, 65.5, 32.8, 16.4};  // LSB/(deg/s)
  double acc_sens_arr_[4] = {16384, 8192, 4096, 2048};  // LSB/g

  // camera and strobe timing
  bool time_init_flag_ = true;
  std::deque<double> time_offset_vec_;
  double time_offset_ = 0.0;
  const std::size_t max_time_offset_samples_ = 1000;
  int init_count_ = 0;

  // camera and strobe count
  uint8_t strobe_count_last_ = 0;
  unsigned int strobe_count_total_ = std::numeric_limits<unsigned int>::infinity();
  unsigned int strobe_count_offset_ = 0;

  // hid usb packet sizes
  const int imu_data_size = 6;  // (int16_t) [ax, ay, az, gx, gy, gz]
  const int imu_buffer_size = 10;  // store 10 samples (imu_stamp, imu_data) in circular buffers
  const int imu_packet_size = 16;  // (int8_t) [imu_stamp[0], ... , imu_stamp[3], imu_data[0], ... , imu_data[11]]
  const int strobe_buffer_size = 10;  // store 10 samples (strobe_stamp, strobe_count) in circular buffers
  const int strobe_packet_size = 5;  // (int8_t) [strobe_stamp[0], ... , strobe_stamp[3], strobe_count]
  const int send_buffer_size = 64;  // (int8_t) size of HID USB packets
  const int send_header_size = 4;  // (int8_t) [send_count[0], send_count[1], imu_count, strobe_count];

  // hid usb packet indices
  const int send_count_index = 0;
  const int imu_count_index = 2;
  const int strobe_count_index = 3;
  const int imu_index[3] = {4, 20, 36};
  const int strobe_index[1] = {52};
  const int timestamp_index = 57;
  const int checksum_index = 62;

  // debug
  bool print_buffer_ = false;
};

}  // namespace svis
