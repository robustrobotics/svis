// Copyright 2017 Massachusetts Institute of Technology

#include "svis_ros/svis_ros.h"

namespace svis_ros {

volatile std::sig_atomic_t SVISRos::stop_signal_ = 0;
  
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

  // setup PublishCamera handler
  auto publish_camera_handler = std::bind(&SVISRos::PublishCamera, this,
                                              std::placeholders::_1);
  svis_.SetPublishCameraHandler(publish_camera_handler);

  // setup PublishTiming handler
  auto publish_timing_handler = std::bind(&SVISRos::PublishTiming, this,
                                              std::placeholders::_1);
  svis_.SetPublishTimingHandler(publish_timing_handler);

  // setup TimeNow handler
  auto time_now_handler = std::bind(&SVISRos::TimeNow, this);
  svis_.SetTimeNowHandler(time_now_handler);
}

void SVISRos::Run() {
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
  while (ros::ok() && !stop_signal_) {
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

  // check if the number of image and info topics is the same
  if (image_topics_.size() != info_topics_.size()) {
    ROS_ERROR("(svis_ros) The number of image topics does not equal the number of info topics. Exiting.");
    exit(1);
  }

  // print image/info topics for user to check pairing
  ROS_WARN("(svis_ros) The following associated image/info pairings must be viable inputs");
  for (int i = 0; i < image_topics_.size(); i++) {
    ROS_INFO("(svis_ros) Image/Info Input %i: {%s, %s}", i, image_topics_[i].c_str(), info_topics_[i].c_str());
  }
}

void SVISRos::InitSubscribers() {
  // message_filters::Subscriber<Image> image_sub(nh, "image", 1);
  // message_filters::Subscriber<CameraInfo> info_sub(nh, "camera_info", 1);
  // TimeSynchronizer<Image, CameraInfo> sync(image_sub, info_sub, 10);
  // sync.registerCallback(boost::bind(&callback, _1, _2));
}

void SVISRos::InitPublishers() {
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/svis/imu", 1);
  svis_imu_pub_ = nh_.advertise<svis_ros::SvisImu>("/svis/imu_packet", 1);
  svis_strobe_pub_ = nh_.advertise<svis_ros::SvisStrobe>("/svis/strobe_packet", 1);
  svis_timing_pub_ = nh_.advertise<svis_ros::SvisTiming>("/svis/timing", 1);
}

void SVISRos::PublishImu(const svis::ImuPacket& imu_packet) {
  svis_.tic();
  sensor_msgs::Imu imu;

  imu.header.stamp = ros::Time(imu_packet.timestamp_ros);
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
  imu.angular_velocity.x = imu_packet.gyro[0];
  imu.angular_velocity.y = imu_packet.gyro[1];
  imu.angular_velocity.z = imu_packet.gyro[2];

  // angular velocity covariance
  for (int i = 0; i < imu.angular_velocity_covariance.size(); i++) {
    imu.angular_velocity_covariance[i] = std::numeric_limits<double>::quiet_NaN();
  }

  // linear acceleration [m/s^2]
  imu.linear_acceleration.x = imu_packet.acc[0];
  imu.linear_acceleration.y = imu_packet.acc[1];
  imu.linear_acceleration.z = imu_packet.acc[2];

  // acceleration covariance
  for (int i = 0; i < imu.linear_acceleration_covariance.size(); i++) {
    imu.linear_acceleration_covariance[i] = std::numeric_limits<double>::quiet_NaN();
  }

  // publish
  imu_pub_.publish(imu);
  //ROS_INFO("stamp: %f, acc.z: %f", imu.header.stamp.toSec(), imu.linear_acceleration.z);

  svis_.timing_.publish_imu = svis_.toc();
}

std::shared_ptr<svis::Image> SVISRos::RosImageToSvis(const sensor_msgs::Image& ros_image) {
  auto svis_image_ptr = std::make_shared<svis::Image>();

  // header
  svis_image_ptr->header.seq = ros_image.header.seq;
  svis_image_ptr->header.stamp = ros_image.header.stamp.toSec();
  svis_image_ptr->header.frame_id = ros_image.header.frame_id;

  // data
  svis_image_ptr->height = ros_image.height;
  svis_image_ptr->width = ros_image.width;
  svis_image_ptr->encoding = ros_image.encoding;
  svis_image_ptr->is_bigendian = ros_image.is_bigendian;
  svis_image_ptr->step = ros_image.step;
  svis_image_ptr->data = ros_image.data;

  return svis_image_ptr;
}

const std::shared_ptr<sensor_msgs::Image> SVISRos::SvisToRosImage(const svis::Image& svis_image) {
  auto ros_image_ptr = std::make_shared<sensor_msgs::Image>();

  // header
  ros_image_ptr->header.seq = svis_image.header.seq;
  ros_image_ptr->header.stamp.fromSec(svis_image.header.stamp);
  ros_image_ptr->header.frame_id = svis_image.header.frame_id;

  // data
  ros_image_ptr->height = svis_image.height;
  ros_image_ptr->width = svis_image.width;
  ros_image_ptr->encoding = svis_image.encoding;
  ros_image_ptr->is_bigendian = svis_image.is_bigendian;
  ros_image_ptr->step = svis_image.step;
  ros_image_ptr->data = svis_image.data;

  return ros_image_ptr;
}

std::shared_ptr<svis::CameraInfo> SVISRos::RosCameraInfoToSvis(const sensor_msgs::CameraInfo& ros_info) {
  auto svis_info_ptr = std::make_shared<svis::CameraInfo>();

  // header
  svis_info_ptr->header.seq = ros_info.header.seq;
  svis_info_ptr->header.stamp = ros_info.header.stamp.toSec();
  svis_info_ptr->header.frame_id = ros_info.header.frame_id;

  // data
  svis_info_ptr->height = ros_info.height;
  svis_info_ptr->width = ros_info.width;
  svis_info_ptr->distortion_model = ros_info.distortion_model;
  svis_info_ptr->D = ros_info.D;
  std::copy(ros_info.K.begin(), ros_info.K.end(), svis_info_ptr->K.begin());
  std::copy(ros_info.R.begin(), ros_info.R.end(), svis_info_ptr->R.begin());
  std::copy(ros_info.P.begin(), ros_info.P.end(), svis_info_ptr->P.begin());
  svis_info_ptr->binning_x = ros_info.binning_x;
  svis_info_ptr->binning_y = ros_info.binning_y;

  return svis_info_ptr;
}

const std::shared_ptr<sensor_msgs::CameraInfo> SVISRos::SvisToRosCameraInfo(const svis::CameraInfo& svis_info) {
  auto ros_info_ptr = std::make_shared<sensor_msgs::CameraInfo>();

  // header
  ros_info_ptr->header.seq = svis_info.header.seq;
  ros_info_ptr->header.stamp.fromSec(svis_info.header.stamp);
  ros_info_ptr->header.frame_id = svis_info.header.frame_id;

  // data
  ros_info_ptr->height = svis_info.height;
  ros_info_ptr->width = svis_info.width;
  ros_info_ptr->distortion_model = svis_info.distortion_model;
  ros_info_ptr->D = svis_info.D;
  std::copy(svis_info.K.begin(), svis_info.K.end(), ros_info_ptr->K.begin());
  std::copy(svis_info.R.begin(), svis_info.R.end(), ros_info_ptr->R.begin());
  std::copy(svis_info.P.begin(), svis_info.P.end(), ros_info_ptr->P.begin());
  ros_info_ptr->binning_x = svis_info.binning_x;
  ros_info_ptr->binning_y = svis_info.binning_y;

  return ros_info_ptr;
}

void SVISRos::CameraCallback(const sensor_msgs::Image::ConstPtr& image_msg,
                             const sensor_msgs::CameraInfo::ConstPtr& info_msg) {
  if (!received_camera_) {
    received_camera_ = true;
  }

  svis::CameraPacket camera_packet;

  // convert image and info
  auto svis_image_ptr = RosImageToSvis(*image_msg);
  auto svis_info_ptr = RosCameraInfoToSvis(*info_msg);

  // metadata
  // PrintMetaDataRaw(image_msg);
  svis_.ParseImageMetadata(*svis_image_ptr, &camera_packet);
  // ROS_INFO("frame_count: %u", camera_packet.metadata.frame_counter);

  // set image and info
  camera_packet.image = *svis_image_ptr;
  camera_packet.info = *svis_info_ptr;

  // add to buffer
  svis_.PushCameraPacket(camera_packet);

  // warn if buffer is at max size
  if (svis_.GetCameraBufferSize() == svis_.GetCameraBufferMaxSize() && !svis_.GetSyncFlag()) {
    ROS_WARN("(svis_ros) camera buffer at max size");
  }
}

void SVISRos::PublishCamera(std::vector<svis::CameraStrobePacket>& camera_strobe_packets) {
  svis_.tic();

  for (int i = 0; i < camera_strobe_packets.size(); i++) {
    // convert
    auto ros_info_ptr = SvisToRosCameraInfo(camera_strobe_packets[i].camera.info);
    auto ros_image_ptr = SvisToRosImage(camera_strobe_packets[i].camera.image);

    // publish
    // camera_pub_.publish(*ros_image_ptr,
    //                     *ros_info_ptr,
    //                     ros::Time(camera_strobe_packets[i].strobe.timestamp_ros));
  }

  svis_.timing_.publish_camera = svis_.toc();
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

void SVISRos::PublishStrobeRaw(const std::vector<svis::StrobePacket>& strobe_packets) {
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
