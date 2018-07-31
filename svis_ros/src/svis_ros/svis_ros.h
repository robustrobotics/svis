// Copyright 2016 Massachusetts Institute of Technology

#pragma once

#include <csignal>
#include <limits>
#include <termios.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <shared_msgs/ImageMetadata.h>

#include "svis/svis.h"
#include "svis/imu_packet.h"
#include "svis/strobe_packet.h"
#include "svis/image.h"
#include "svis/camera_info.h"
#include "svis_ros/SvisImu.h"
#include "svis_ros/SvisStrobe.h"
#include "svis_ros/SvisTiming.h"

namespace svis_ros {

template <typename T>
inline bool SafeGetParam(ros::NodeHandle & nh,
                         std::string const& param_name,
                         T & param_value) {
  if (!nh.getParam(param_name, param_value)) {
    ROS_ERROR("Failed to find parameter: %s", nh.resolveName(param_name, true).c_str());
    exit(1);
  }
  return true;
}

class SVISRos {
 public:
  SVISRos();
  void Run();

 private:
  void GetParams();
  void InitSubscribers();
  void InitPublishers();

  // callbacks
  void CameraSyncCallback(const sensor_msgs::Image::ConstPtr& image_msg,
                          const sensor_msgs::CameraInfo::ConstPtr& info_msg,
                          const shared_msgs::ImageMetadata::ConstPtr& metadata_msg);

  // publishers
  void PublishImuRaw(const std::vector<svis::ImuPacket>& imu_packets);
  void PublishImu(const svis::ImuPacket& imu_packet);
  void PublishStrobeRaw(const std::vector<svis::StrobePacket>& strobe_packets);
  void PublishTiming(const svis::Timing& timing);
  double TimeNow();

  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // publishers
  ros::Publisher imu_pub_;
  ros::Publisher svis_imu_pub_;
  ros::Publisher svis_strobe_pub_;
  ros::Publisher svis_timing_pub_;
  std::map<std::string, ros::Publisher> image_pubs_;
  std::map<std::string, ros::Publisher> info_pubs_;
  std::map<std::string, ros::Publisher> metadata_pubs_;
  std::vector<std::string> sensor_names_;

  // subscribers
  std::vector<std::string> image_topics_;
  std::vector<std::string> info_topics_;
  std::vector<std::string> metadata_topics_;
  std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>> image_sub_ptrs_;
  std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>>> info_sub_ptrs_;
  std::vector<std::shared_ptr<message_filters::Subscriber<shared_msgs::ImageMetadata>>> metadata_sub_ptrs_;
  std::vector<std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image,
                                                                sensor_msgs::CameraInfo,
                                                                shared_msgs::ImageMetadata>>> camera_sync_ptrs_;

  bool received_camera_ = false;
  svis::SVIS svis_;
};

}  // namespace svis_ros
