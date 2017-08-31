// Copyright 2017 Massachusetts Institute of Technology

#include "svis_ros/svis_ros.h"

namespace svis_ros {

volatile std::sig_atomic_t SVISRos::stop_signal_ = 0;
  
SVISRos::SVISRos()
  : nh_(),
    pnh_("~"),
    it_(nh_) {
  // nothing
}

void SVISRos::Run() {
  GetParams();
  InitSubscribers();
  InitPublishers();
  ConfigureCamera();

  // setup comms and send init packet
  svis_.OpenHID();

  // send setup packet
  svis_.SendSetup();

  ros::Rate r(1000);
  while (ros::ok() && !stop_signal_) {
    t_period_ = ros::Time::now();
    timing_.period = (t_period_ - t_period_last_).toSec();
    t_period_last_ = t_period_;

    tic();
    ros::spinOnce();
    svis_.timing_.ros_spin_once = toc();

    svis_.Update();

    if (svis_.use_camera_ && !received_camera_) {
      ROS_WARN_THROTTLE(0.5, "(svis_ros) Have not received camera message");
    }

    r.sleep();
  }
}  

void SVISRos::ConfigureCamera() {
  ROS_INFO("Configuring camera.");
  ROS_WARN("Make sure camera driver is running.");

  // toggle pointgrey trigger mode
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::StrParameter trigger_mode;
  // dynamic_reconfigure::Config conf;

  trigger_mode.name = "trigger_mode";
  trigger_mode.value = "mode1";
  srv_req.config.strs.push_back(trigger_mode);

  // set param and check for success
  bool param_set = false;
  ros::Rate r(10);
  while (!param_set) {
    // set trigger mode
    ros::service::call("/flea3/camera_nodelet/set_parameters", srv_req, srv_resp);

    // check for success
    for (int i = 0; i < srv_resp.config.strs.size(); i++) {
      // ROS_INFO("name: %s", srv_resp.config.strs[i].name.c_str());
      // ROS_INFO("value: %s", srv_resp.config.strs[i].value.c_str());
      if (srv_resp.config.strs[i].name == "trigger_mode" && srv_resp.config.strs[i].value == trigger_mode.value) {
        param_set = true;
      }
    }

    r.sleep();
  }

  // reset configure params
  srv_req.config.strs.clear();
  trigger_mode.name = "trigger_mode";
  trigger_mode.value = "mode0";
  srv_req.config.strs.push_back(trigger_mode);

  // set param and check for success
  param_set = false;
  while (!param_set) {
    // set trigger mode
    ros::service::call("/flea3/camera_nodelet/set_parameters", srv_req, srv_resp);

    // check for success
    for (int i = 0; i < srv_resp.config.strs.size(); i++) {
      // ROS_INFO("name: %s", srv_resp.config.strs[i].name.c_str());
      // ROS_INFO("value: %s", srv_resp.config.strs[i].value.c_str());
      if (srv_resp.config.strs[i].name == "trigger_mode" && srv_resp.config.strs[i].value == trigger_mode.value) {
        param_set = true;
      }
    }

    r.sleep();
  }
}

void SVISRos::GetParams() {
  ros::NodeHandle pnh("~");

  fla_utils::SafeGetParam(pnh, "camera_rate", svis_.camera_rate_);
  fla_utils::SafeGetParam(pnh, "gyro_sens", svis_.gyro_sens_);
  fla_utils::SafeGetParam(pnh, "acc_sens", svis_.acc_sens_);
  fla_utils::SafeGetParam(pnh, "imu_filter_size", svis_.imu_filter_size_);
}

void SVISRos::InitSubscribers() {
  camera_sub_ = it_.subscribeCamera("/flea3/image_raw", 10, &SVISRos::CameraCallback, this);
}

void SVISRos::InitPublishers() {
  camera_pub_ = it_.advertiseCamera("/svis/image_raw", 1);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/svis/imu", 1);
  svis_imu_pub_ = nh_.advertise<svis_ros::SvisImu>("/svis/imu_packet", 1);
  svis_strobe_pub_ = nh_.advertise<svis_ros::SvisStrobe>("/svis/strobe_packet", 1);
  svis_timing_pub_ = nh_.advertise<svis_ros::SvisTiming>("/svis/timing", 1);
}

void SVISRos::PublishImu(std::vector<svis::ImuPacket>& imu_packets_filt) {
  svis_.tic();

  svis::ImuPacket temp_packet;
  sensor_msgs::Imu imu;

  for (int i = 0; i < imu_packets_filt.size(); i++) {
    temp_packet = imu_packets_filt[i];

    imu.header.stamp = ros::Time(temp_packet.timestamp_teensy + svis_.time_offset_);
    imu.header.frame_id = "body";

    // orientation
    imu.orientation.x = std::numeric_limits<double>::quiet_NaN();
    imu.orientation.y = std::numeric_limits<double>::quiet_NaN();
    imu.orientation.z = std::numeric_limits<double>::quiet_NaN();
    imu.orientation.w = std::numeric_limits<double>::quiet_NaN();

    // orientation covariance
    for (int i = 0; i < imu.orientation_covariance.size(); i++) {
      imu.orientation_covariance[i] = std::numeric_limits<double>::quiet_NaN();
    }

    // angular velocity [rad/s]
    imu.angular_velocity.x = temp_packet.gyro[0];
    imu.angular_velocity.y = temp_packet.gyro[1];
    imu.angular_velocity.z = temp_packet.gyro[2];

    // angular velocity covariance
    for (int i = 0; i < imu.angular_velocity_covariance.size(); i++) {
      imu.angular_velocity_covariance[i] = std::numeric_limits<double>::quiet_NaN();
    }

    // linear acceleration [m/s^2]
    imu.linear_acceleration.x = temp_packet.acc[0];
    imu.linear_acceleration.y = temp_packet.acc[1];
    imu.linear_acceleration.z = temp_packet.acc[2];

    // acceleration covariance
    for (int i = 0; i < imu.linear_acceleration_covariance.size(); i++) {
      imu.linear_acceleration_covariance[i] = std::numeric_limits<double>::quiet_NaN();
    }

    // publish
    imu_pub_.publish(imu);
  }

  svis_.timing_.publish_imu = svis_.toc();
}

void SVISRos::CameraCallback(const sensor_msgs::Image::ConstPtr& image_msg,
                             const sensor_msgs::CameraInfo::ConstPtr& info_msg) {
  if (!received_camera_) {
    received_camera_ = true;
  }

  // PrintMetaDataRaw(image_msg);
  svis::CameraPacket camera_packet;

  // metadata
  svis_.GetImageMetadata(image_msg, &camera_packet);
  // ROS_INFO("frame_count: %u", camera_packet.metadata.frame_counter);

  // set image and info
  camera_packet.image = *image_msg;
  camera_packet.info = *info_msg;

  // add to buffer
  svis_.camera_buffer_.push_back(camera_packet);

  // warn if buffer is at max size
  if (svis_.camera_buffer_.size() == svis_.camera_buffer_.max_size() && !svis_.sync_flag_) {
    ROS_WARN("(svis_ros) camera buffer at max size");
  }
}

void SVISRos::PublishCamera(std::vector<svis::CameraStrobePacket>& camera_strobe_packets) {
  svis_.tic();

  for (int i = 0; i < camera_strobe_packets.size(); i++) {
    camera_pub_.publish(camera_strobe_packets[i].camera.image,
                        camera_strobe_packets[i].camera.info, ros::Time(camera_strobe_packets[i].strobe.timestamp_ros));
  }

  camera_strobe_packets.clear();

  svis_.timing_.publish_camera = svis_.toc();
}

void SVISRos::PublishImuRaw(std::vector<svis::ImuPacket>& imu_packets) {
  svis_.tic();

  svis_ros::SvisImu imu;

  // check sizes
  if (imu.SIZE != imu_packets.size()) {
    ROS_WARN("(svis_ros) mismatch in packet size");
    return;
  }

  imu.header.stamp = ros::Time::now();
  imu.header.frame_id = "svis_imu_frame";
  for (int i = 0; i < imu_packets.size(); i++) {
    imu.timestamp_ros_rx[i] = imu_packets[i].timestamp_ros_rx;
    imu.timestamp_ros[i] = imu_packets[i].timestamp_ros;
    imu.timestamp_teensy_raw[i] = imu_packets[i].timestamp_teensy_raw;
    imu.timestamp_teensy[i] = imu_packets[i].timestamp_teensy;
    imu.accx[i] = imu_packets[i].acc[0];
    imu.accy[i] = imu_packets[i].acc[1];
    imu.accz[i] = imu_packets[i].acc[2];
    imu.gyrox[i] = imu_packets[i].gyro[0];
    imu.gyroy[i] = imu_packets[i].gyro[1];
    imu.gyroz[i] = imu_packets[i].gyro[2];
  }

  // publish
  svis_imu_pub_.publish(imu);

  svis_.timing_.publish_imu_raw = svis_.toc();
}

void SVISRos::PublishStrobeRaw(std::vector<svis::StrobePacket> &strobe_packets) {
  svis_.tic();

  svis_ros::SvisStrobe strobe;
  for (int i = 0; i < strobe_packets.size(); i++) {
    strobe.header.stamp = ros::Time::now();

    strobe.timestamp_ros_rx = strobe_packets[i].timestamp_ros_rx;
    strobe.timestamp_ros = strobe_packets[i].timestamp_ros;
    strobe.timestamp_teensy_raw = strobe_packets[i].timestamp_teensy_raw;
    strobe.timestamp_teensy = strobe_packets[i].timestamp_teensy;
    strobe.count = strobe_packets[i].count;

    // publish
    svis_strobe_pub_.publish(strobe);
  }

  svis_.timing_.publish_strobe_raw = svis_.toc();
}

void SVISRos::PublishTiming() {
  svis_timing_pub_.publish(svis_.timing_);
}

}  // namespace svis_ros
