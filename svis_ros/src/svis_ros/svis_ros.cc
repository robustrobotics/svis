// Copyright 2017 Massachusetts Institute of Technology

#include "svis_ros/svis_ros.h"

namespace svis_ros {

SVISRos::SVISRos()
  : nh_(),
    pnh_("~") {
  // setup PublishStrobeRaw handler
  auto publish_strobe_raw_handler = std::bind(&SVISRos::PublishStrobeRaw, this,
                                              std::placeholders::_1);
  svis_.SetPublishStrobeRawHandler(publish_strobe_raw_handler);

  // setup PublishImuRaw handler
  auto publish_imu_raw_handler = std::bind(&SVISRos::PublishImuRaw, this,
                                              std::placeholders::_1);
  svis_.SetPublishImuRawHandler(publish_imu_raw_handler);

  // setup PublishImu handler
  auto publish_imu_handler = std::bind(&SVISRos::PublishImu, this,
				       std::placeholders::_1);
  svis_.SetPublishImuHandler(publish_imu_handler);

  // setup PublishTiming handler
  auto publish_timing_handler = std::bind(&SVISRos::PublishTiming, this,
                                              std::placeholders::_1);
  svis_.SetPublishTimingHandler(publish_timing_handler);

  // setup TimeNow handler
  auto time_now_handler = std::bind(&SVISRos::TimeNow, this);
  svis_.SetTimeNowHandler(time_now_handler);
}

void SVISRos::Run() {
  sleep(5);
  
  GetParams();
  InitSubscribers();
  InitPublishers();
  // ConfigureCamera();

  // setup comms and send init packet
  svis_.OpenHID();

  // send setup packet
  svis_.SendSetup();

  ros::Time t_start = ros::Time::now();
  ros::Time t_start_last = t_start;
  ros::Rate r(1000);
  while (ros::ok()) {
    t_start = ros::Time::now();
    svis_.timing_.period = (t_start - t_start_last).toSec();
    t_start_last = t_start;

    svis_.tic();
    ros::spinOnce();
    svis_.timing_.ros_spin_once = svis_.toc();

    svis_.Update();

    if (!received_camera_) {
      ROS_WARN_THROTTLE(0.5, "(svis_ros) Have not received camera message");
    }

    r.sleep();
  }
}  

void SVISRos::GetParams() {
  ros::NodeHandle pnh("~");

  SafeGetParam(pnh, "camera_rate", svis_.camera_rate_);
  SafeGetParam(pnh, "gyro_sens", svis_.gyro_sens_);
  SafeGetParam(pnh, "acc_sens", svis_.acc_sens_);
  SafeGetParam(pnh, "imu_filter_size", svis_.imu_filter_size_);
  SafeGetParam(pnh, "offset_sample_count", svis_.offset_sample_count_);
  SafeGetParam(pnh, "offset_sample_time", svis_.offset_sample_time_);

  SafeGetParam(pnh, "image_topics", image_topics_);
  SafeGetParam(pnh, "info_topics", info_topics_);
  SafeGetParam(pnh, "metadata_topics", metadata_topics_);

  SafeGetParam(pnh, "sensor_names", sensor_names_);

  // check if the number of image and info topics is the same
  if (image_topics_.size() != info_topics_.size() ||
      image_topics_.size() != metadata_topics_.size()) {
    ROS_ERROR("(svis_ros) The number of inputs topics is not equal. Exiting.");
    exit(1);
  }

  // print image/info topics for user to check pairing
  ROS_WARN("(svis_ros) The following associated image/info pairings must be viable inputs");
  for (int i = 0; i < image_topics_.size(); i++) {
    ROS_INFO("(svis_ros) Image/Info/Metadata Input %i: {%s, %s, %s}",
             i,
             image_topics_[i].c_str(),
             info_topics_[i].c_str(),
             metadata_topics_[i].c_str());
  }
}

void SVISRos::InitSubscribers() {
  for (int i = 0; i < image_topics_.size(); i++) {
    auto image_sub_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(pnh_, image_topics_[i], 10);
    image_sub_ptrs_.push_back(image_sub_ptr);

    auto info_sub_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo>>(pnh_, info_topics_[i], 10);
    info_sub_ptrs_.push_back(info_sub_ptr);

    auto metadata_sub_ptr = std::make_shared<message_filters::Subscriber<shared_msgs::ImageMetadata>>(pnh_, metadata_topics_[i], 10);
    metadata_sub_ptrs_.push_back(metadata_sub_ptr);

    auto camera_sync_ptr = std::make_shared<message_filters::TimeSynchronizer<
                                              sensor_msgs::Image,
                                              sensor_msgs::CameraInfo,
                                              shared_msgs::ImageMetadata>>(*image_sub_ptr,
                                                                           *info_sub_ptr,
                                                                           *metadata_sub_ptr,
                                                                           10);
    camera_sync_ptr->registerCallback(boost::bind(&SVISRos::CameraSyncCallback, this, _1, _2, _3));
    camera_sync_ptrs_.push_back(camera_sync_ptr);
  }
}

void SVISRos::InitPublishers() {
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/svis/imu", 10);
  svis_imu_pub_ = nh_.advertise<svis_ros::SvisImu>("/svis/imu_packet", 10);
  svis_strobe_pub_ = nh_.advertise<svis_ros::SvisStrobe>("/svis/strobe_packet", 10);
  svis_timing_pub_ = nh_.advertise<svis_ros::SvisTiming>("/svis/timing", 10);

  for (int i = 0; i < image_topics_.size(); i++) {
    image_pubs_[sensor_names_[i]] = nh_.advertise<sensor_msgs::Image>("/svis/" + sensor_names_[i] + "/image_raw", 10);
    info_pubs_[sensor_names_[i]] = nh_.advertise<sensor_msgs::CameraInfo>("/svis/" + sensor_names_[i] + "/camera_info", 10);
    metadata_pubs_[sensor_names_[i]] = nh_.advertise<shared_msgs::ImageMetadata>("/svis/" + sensor_names_[i] + "/metadata", 10);
  }
}

void SVISRos::PublishImu(const svis::ImuPacket& imu_packet) {
  svis_.tic();
  sensor_msgs::Imu imu;

  imu.header.stamp = ros::Time(imu_packet.timestamp_ros);
  imu.header.frame_id = "svis_imu";

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
  imu.angular_velocity.x = imu_packet.gyro[2];
  imu.angular_velocity.y = -imu_packet.gyro[1];
  imu.angular_velocity.z = imu_packet.gyro[0];

  // angular velocity covariance
  for (int i = 0; i < imu.angular_velocity_covariance.size(); i++) {
    imu.angular_velocity_covariance[i] = std::numeric_limits<double>::quiet_NaN();
  }

  // linear acceleration [m/s^2]
  imu.linear_acceleration.x = imu_packet.acc[2];
  imu.linear_acceleration.y = -imu_packet.acc[1];
  imu.linear_acceleration.z = imu_packet.acc[0];

  // acceleration covariance
  for (int i = 0; i < imu.linear_acceleration_covariance.size(); i++) {
    imu.linear_acceleration_covariance[i] = std::numeric_limits<double>::quiet_NaN();
  }

  // publish
  imu_pub_.publish(imu);
  //ROS_INFO("stamp: %f, acc.z: %f", imu.header.stamp.toSec(), imu.linear_acceleration.z);

  svis_.timing_.publish_imu = svis_.toc();
}

void SVISRos::CameraSyncCallback(const sensor_msgs::Image::ConstPtr& image_msg,
                                 const sensor_msgs::CameraInfo::ConstPtr& info_msg,
                                 const shared_msgs::ImageMetadata::ConstPtr& metadata_msg) {
  // static double time_offset = metadata_msg->header.stamp.toSec() - static_cast<double>(metadata_msg->sensor_timestamp) * 1e-6;
  
  if (!received_camera_) {
    received_camera_ = true;
  }

  // copy metadata
  svis::ImageMetadata metadata;
  metadata.sensor_name = metadata_msg->sensor_name.data;
  metadata.frame_counter = metadata_msg->frame_counter;
  metadata.frame_timestamp = metadata_msg->frame_timestamp;
  metadata.sensor_timestamp = metadata_msg->sensor_timestamp;
  metadata.exposure_time = metadata_msg->exposure_time;
  
  // create camera packet
  svis::CameraPacket camera_packet;
  camera_packet.timestamp = image_msg->header.stamp.toSec();
  camera_packet.metadata = metadata;

  // add to buffer
  svis_.PushCameraPacket(camera_packet);

  if (!svis_.Synchronized()) {
    return;
  }

  double synchronized_timestamp;
  // ROS_INFO("[SVISRos::CameraSyncCallback] Getting timestamp for frame %lu of sensor %s", metadata.frame_counter, metadata.sensor_name.c_str());
  if (svis_.GetSynchronizedTime(metadata.sensor_name, metadata.frame_counter, &synchronized_timestamp)) {
    ROS_INFO("[SVISRos::CameraSyncCallback] Got synchronzied timestamp: %f, camera timestamp: %f, diff: %f",
	     synchronized_timestamp,
	     camera_packet.timestamp,
	     synchronized_timestamp - camera_packet.timestamp);
    ros::Time stamp(synchronized_timestamp);

    sensor_msgs::Image sync_image = *image_msg;
    sync_image.header.stamp = stamp;

    // build sync info
    sensor_msgs::CameraInfo sync_info = *info_msg;
    sync_info.header.stamp = stamp;

    // build sync metadata
    shared_msgs::ImageMetadata sync_metadata = *metadata_msg;
    sync_metadata.header.stamp = stamp;

    // publish all
    image_pubs_[metadata.sensor_name].publish(sync_image);
    info_pubs_[metadata.sensor_name].publish(sync_info);
    metadata_pubs_[metadata.sensor_name].publish(sync_metadata);
  }

  // ROS_INFO("[SVISRos::CameraSyncCallback] Exiting");
}

void SVISRos::PublishImuRaw(const std::vector<svis::ImuPacket>& imu_packets) {
  svis_.tic();

  svis_ros::SvisImu imu;

  // check sizes
  if (imu.SIZE != imu_packets.size()) {
    ROS_WARN("(svis_ros) mismatch in packet size");
    return;
  }

  imu.header.stamp = ros::Time::now();
  imu.header.frame_id = "svis_imu";
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

void SVISRos::PublishStrobeRaw(const std::vector<svis::StrobePacket>& strobe_packets) {
  svis_.tic();

  svis_ros::SvisStrobe strobe;
  for (int i = 0; i < strobe_packets.size(); i++) {
    strobe.header.stamp = ros::Time(strobe_packets[i].timestamp_ros);

    strobe.timestamp_ros_rx = strobe_packets[i].timestamp_ros_rx;
    strobe.timestamp_ros = strobe_packets[i].timestamp_ros;
    strobe.timestamp_teensy_raw = strobe_packets[i].timestamp_teensy_raw;
    strobe.timestamp_teensy = strobe_packets[i].timestamp_teensy;
    strobe.count = strobe_packets[i].count;
    strobe.count_total = strobe_packets[i].count_total;

    // publish
    svis_strobe_pub_.publish(strobe);
  }

  svis_.timing_.publish_strobe_raw = svis_.toc();
}

void SVISRos::PublishTiming(const svis::Timing& timing) {
  SvisTiming msg;

  msg.header.stamp = ros::Time::now();

  msg.rawhid_recv = timing.rawhid_recv;
  msg.ros_spin_once = timing.ros_spin_once;
  msg.check_checksum = timing.check_checksum;
  msg.parse_header = timing.parse_header;
  msg.parse_imu = timing.parse_imu;
  msg.parse_strobe = timing.parse_strobe;
  msg.compute_strobe_total = timing.compute_strobe_total;
  msg.publish_imu_raw = timing.publish_imu_raw;
  msg.publish_strobe_raw = timing.publish_strobe_raw;
  msg.push_imu = timing.push_imu;
  msg.push_strobe = timing.push_strobe;
  msg.compute_offsets = timing.compute_offsets;
  msg.filter_imu = timing.filter_imu;
  msg.publish_imu = timing.publish_imu;
  msg.associate = timing.associate;
  msg.publish_camera = timing.publish_camera;
  msg.update = timing.update;
  msg.period = timing.period;

  svis_timing_pub_.publish(msg);
}

double SVISRos::TimeNow() {
  return ros::Time::now().toSec();
}

}  // namespace svis_ros
