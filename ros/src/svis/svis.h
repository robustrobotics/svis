// Copyright 2016 Massachusetts Institute of Technology

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "svis/header_packet.h"
#include "svis/imu_packet.h"
#include "svis/strobe_packet.h"
#include "svis/camera_packet.h"
#include "svis/camera_strobe_packet.h"

extern "C" {
#include "hid/hid.h"
}

namespace svis {

class SVIS {
 public:
  SVIS();

  void Update();
  void InitHID();
  void ReadHID(std::vector<char>* buf);
  void ParseBuffer(const std::vector<char>& buf,
                      std::vector<ImuPacket>* imu_packets,
                      std::vector<StrobePacket>* strobe_packets);
  void SendPulse();
  void SendDisablePulse();
  void SendSetup();
  bool CheckChecksum(const std::vector<char>& buf);
  void GetTimeOffset();
  void GetHeader(const std::vector<char>& buf,
                 HeaderPacket* header);
  void GetImu(const std::vector<char>& buf,
              const HeaderPacket& header,
              std::vector<ImuPacket>* imu_packets);
  void GetStrobe(const std::vector<char>& buf,
                 const HeaderPacket& header,
                 std::vector<StrobePacket>* strobe_packets);
  void PushImu(const std::vector<ImuPacket>& imu_packets);
  void PushStrobe(const std::vector<StrobePacket>& strobe_packets);
  void FilterImu(std::vector<ImuPacket>* imu_packets_filt);
  void PrintBuffer(const std::vector<char>& buf);
  void PrintImageQuadlet(const std::string& name,
                         const sensor_msgs::Image::ConstPtr& msg,
                         const int& i);
  void PrintMetaDataRaw(const sensor_msgs::Image::ConstPtr& msg);
  void GetStrobeTotal(std::vector<StrobePacket>* strobe_packets);
  void GetCountOffset();
  void AssociateStrobe(std::vector<CameraStrobePacket>* camera_strobe_packets);
  void PrintCameraBuffer();
  void PrintStrobeBuffer();
  void tic();
  double toc();
  void GetImageMetadata(const sensor_msgs::Image::ConstPtr& image_msg,
                        CameraPacket* camera_packet);

  // buffers
  boost::circular_buffer<ImuPacket> imu_buffer_;
  boost::circular_buffer<StrobePacket> strobe_buffer_;
  boost::circular_buffer<CameraPacket> camera_buffer_;
  boost::circular_buffer<CameraStrobePacket> camera_strobe_buffer_;

  // configuration
  bool use_camera_ = true;
  int camera_rate_ = 0;

  // constants
  const double g_ = 9.80665;
  const double rad_per_deg_ = 0.0174533;

  // imu
  int imu_filter_size_ = 0;
  double gyro_sens_arr_[4] = {131, 65.5, 32.8, 16.4};  // LSB/(deg/s)
  double acc_sens_arr_[4] = {16384, 8192, 4096, 2048};  // LSB/g
  int gyro_sens_ = 0;  // gyro sensitivity selection [0,3]
  int acc_sens_ = 0;  // acc sensitivity selection [0,3]

  // camera and strobe timing
  bool init_flag_ = true;
  bool sent_pulse_ = false;
  std::deque<double> time_offset_vec_;
  double time_offset_ = 0.0;
  int init_count_ = 0;

  // camera and strobe count
  bool sync_flag_ = true;
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
  const int strobe_index[2] = {52, 57};
  const int checksum_index = 62;

  // debug
  bool print_buffer_ = false;

  // timing
  ros::Time t_loop_start_;
  ros::Time t_period_;
  ros::Time t_period_last_;
  ros::Time t_pulse_;
  ros::Time tic_;
  ros::Time toc_;
};

}  // namespace svis_ros
