// Copyright 2016 Massachusetts Institute of Technology

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <csignal>
#include <boost/circular_buffer.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include "fla_utils/param_utils.h"

#include "svis_ros/imu_packet.h"
#include "svis_ros/header_packet.h"
#include "svis_ros/strobe_packet.h"
#include "svis_ros/camera_packet.h"
#include "svis_ros/image_metadata.h"
#include "svis_ros/camera_strobe_packet.h"
#include "svis_ros/SvisImu.h"
#include "svis_ros/SvisStrobe.h"
#include "svis_ros/SvisTiming.h"

extern "C" {
#include "hid/hid.h"
}

namespace svis_ros {

class SVISRos {
 public:
  SVISRos();
  void Run();

  static volatile std::sig_atomic_t stop_signal_;

 private:
  void PublishCamera(std::vector<CameraStrobePacket> &camera_strobe_packets);
  void GetParams();
  void InitSubscribers();
  void InitPublishers();
  void CameraCallback(const sensor_msgs::Image::ConstPtr& image_msg,
                      const sensor_msgs::CameraInfo::ConstPtr& info_msg);
  void PublishImuRaw(std::vector<ImuPacket> &imu_packets);
  void PublishImu(std::vector<ImuPacket> &imu_packets_filt);
  void PublishStrobeRaw(std::vector<StrobePacket> &strobe_packets);
  void PublishTiming();
  
  void ConfigureCamera();
  void SendPulse();
  void SendDisablePulse();
  void SendSetup();
  int GetChecksum(std::vector<char> &buf);
  void GetTimeOffset();
  void GetHeader(std::vector<char> &buf, HeaderPacket &header);
  void GetImu(std::vector<char> &buf, HeaderPacket &header, std::vector<ImuPacket> &imu_packets);
  void GetStrobe(std::vector<char> &buf, HeaderPacket &header,
                          std::vector<StrobePacket> &strobe_packets);
  void PushImu(std::vector<ImuPacket> &imu_packets);
  void PushStrobe(std::vector<StrobePacket> &strobe_packets);
  void FilterImu(std::vector<ImuPacket> &imu_packets_filt);
  void PrintBuffer(std::vector<char> &buf);
  void PrintImageQuadlet(std::string name, const sensor_msgs::Image::ConstPtr& msg, int i);
  void PrintMetaDataRaw(const sensor_msgs::Image::ConstPtr& msg);
  void GetImageMetadata(const sensor_msgs::Image::ConstPtr& image_msg,
                                 CameraPacket &camera_packet);
  void GetStrobeTotal(std::vector<StrobePacket> &strobe_packets);
  void GetCountOffset();
  void AssociateStrobe(std::vector<CameraStrobePacket> &camera_strobe_packets);
  void PrintCameraBuffer();
  void PrintStrobeBuffer();
  void tic();
  double toc();

  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;

  // publishers
  image_transport::CameraPublisher camera_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher svis_imu_pub_;
  ros::Publisher svis_strobe_pub_;
  ros::Publisher svis_timing_pub_;

  // subscribers
  image_transport::CameraSubscriber camera_sub_;

  // buffers
  boost::circular_buffer<ImuPacket> imu_buffer_;
  boost::circular_buffer<StrobePacket> strobe_buffer_;
  boost::circular_buffer<CameraPacket> camera_buffer_;
  boost::circular_buffer<CameraStrobePacket> camera_strobe_buffer_;

  // configuration
  bool use_camera_ = true;
  bool received_camera_ = false;
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
  ros::Time t_loop_start_;
  ros::Time t_period_;
  ros::Time t_period_last_;
  ros::Time t_pulse_;
  ros::Time tic_;
  ros::Time toc_;
  svis_ros::SvisTiming timing_;
};

}  // namespace svis_ros
