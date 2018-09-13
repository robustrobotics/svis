// Copyright 2016 Massachusetts Institute of Technology

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "svis_ros/svis_ros.h"

namespace svis_ros {

class SVISRosNodelet : public nodelet::Nodelet {
 public:
  virtual void onInit() {
    SVISRos svis_ros;
    svis_ros.Run();

    return;
  }
};

}  // namespace svis_ros

// Export as a plugin.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(svis_ros::SVISRosNodelet, nodelet::Nodelet)
