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

  fla_utils::SafeGetParam(pnh, "camera_rate", camera_rate_);
  fla_utils::SafeGetParam(pnh, "gyro_sens", gyro_sens_);
  fla_utils::SafeGetParam(pnh, "acc_sens", acc_sens_);
  fla_utils::SafeGetParam(pnh, "imu_filter_size", imu_filter_size_);
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

/**
 * \brief Main processing loop.
 */
void SVISRos::Run() {
  GetParams();
  InitSubscribers();
  InitPublishers();
  ConfigureCamera();

  // set circular buffer max lengths
  imu_buffer_.set_capacity(10);
  strobe_buffer_.set_capacity(10);
  camera_buffer_.set_capacity(20);
  camera_strobe_buffer_.set_capacity(10);

  // open rawhid port
  int r = rawhid_open(1, 0x16C0, 0x0486, 0xFFAB, 0x0200);
  // C-based example is 16C0:0480:FFAB:0200
  // Arduino-based example is 16C0:0486:FFAB:0200

  // check return
  if (r <= 0) {
    ROS_ERROR("(svis_ros) No svis_teensy device found.\n");
    return;
  } else {
    ROS_INFO("(svis_ros) Found svis_teensy device\n");
  }

  // send setup packet
  SendSetup();

  // loop
  std::vector<char> buf(64, 0);
  // while (ros::ok() && !stop_signal_) {
  while (ros::ok()) {
    // period
    t_period_ = ros::Time::now();
    timing_.period = (t_period_ - t_period_last_).toSec();
    t_period_last_ = t_period_;

    // check if any Raw HID packet has arrived
    tic();
    int num = rawhid_recv(0, buf.data(), buf.size(), 220);
    timing_.rawhid_recv = toc();

    // check byte count
    if (num < 0) {
      ROS_ERROR("(svis_ros): Error reading, device went offline");
      rawhid_close(0);
      return;
    } else if (num == 0) {
      if (!init_flag_) {
        ROS_INFO("(svis_ros) 0 bytes received");
      }
    } else if (num > 0) {
      t_loop_start_ = ros::Time::now();

      // spin
      tic();
      ros::spinOnce();
      timing_.ros_spin_once = toc();

      // resize vector
      buf.resize(num);

      // debug print
      if (print_buffer_) {
        PrintBuffer(buf);
      }

      // bail if checksums don't match
      if (GetChecksum(buf)) {
        continue;
      }

      // parse header
      HeaderPacket header;
      GetHeader(buf, header);

      // parse, publish, push imu
      std::vector<ImuPacket> imu_packets;
      GetImu(buf, header, imu_packets);
      PublishImuRaw(imu_packets);
      PushImu(imu_packets);

      // parse, publish, push strobe
      std::vector<StrobePacket> strobe_packets;
      GetStrobe(buf, header, strobe_packets);
      GetStrobeTotal(strobe_packets);
      PublishStrobeRaw(strobe_packets);
      PushStrobe(strobe_packets);

      // get difference between ros and teensy epochs
      if (init_flag_) {
        GetTimeOffset();
        continue;
      }

      // filter and publish imu
      std::vector<ImuPacket> imu_packets_filt;
      FilterImu(imu_packets_filt);
      PublishImu(imu_packets_filt);

      // sync the camera and strobe counts
      // if (sync_flag_) {
      //   GetCountOffset();
      //   continue;
      // }

      // PrintStrobeBuffer();
      // PrintCameraBuffer();

      // associate strobe with camera and publish
      std::vector<CameraStrobePacket> camera_strobe_packets;
      AssociateStrobe(camera_strobe_packets);
      PublishCamera(camera_strobe_packets);

      timing_.loop = (ros::Time::now() - t_loop_start_).toSec();
      PublishTiming();

      if (use_camera_ && !received_camera_) {
        ROS_WARN_THROTTLE(0.5, "(svis_ros) Have not received camera message");
      }
    } else {
      ROS_WARN("(svis_ros) Bad return value from rawhid_recv");
    }
  }
}

void SVISRos::SendPulse() {
  std::vector<char> buf(64, 0);
  buf[0] = 0xAB;
  buf[1] = 2;
  ROS_INFO("(svis_ros) Sending pulse packet");
  rawhid_send(0, buf.data(), buf.size(), 100);
  sent_pulse_ = true;
  t_pulse_ = ros::Time::now();
}

void SVISRos::SendDisablePulse() {
  std::vector<char> buf(64, 0);
  buf[0] = 0xAB;
  buf[1] = 3;
  ROS_INFO("(svis_ros) Sending configuration packet");
  rawhid_send(0, buf.data(), buf.size(), 100);
}

void SVISRos::SendSetup() {
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

  ROS_INFO("(svis_ros) Sending configuration packet");
  rawhid_send(0, buf.data(), buf.size(), 100);
}

int SVISRos::GetChecksum(std::vector<char> &buf) {
  tic();

  // calculate checksum
  uint16_t checksum_calc = 0;
  for (int i = 0; i < send_buffer_size - 2; i++) {
    checksum_calc += buf[i] & 0xFF;
  }

  // get packet chechsum
  uint16_t checksum_orig = 0;
  memcpy(&checksum_orig, &buf[checksum_index], sizeof(checksum_orig));
  int ret = (checksum_calc != checksum_orig);

  // check match and return result
  if (ret) {
    ROS_INFO("(svis_ros) checksum error [%02X, %02X] [%02X, %02X]",
                 buf[checksum_index] & 0xFF, buf[checksum_index + 1] & 0xFF, checksum_calc, checksum_orig);
  }

  timing_.get_checksum = toc();

  return ret;
}

void SVISRos::GetTimeOffset() {
    if (time_offset_vec_.size() >= 100) {
      // turn off camera pulse
      SendDisablePulse();

      // filter initial values that are often composed of stale data
      // ROS_INFO("(svis_ros) time_offset_vec.size(): %lu", time_offset_vec_.size());
      while (fabs(time_offset_vec_.front() - time_offset_vec_.back()) > 0.1) {
        time_offset_vec_.pop_front();
      }
      // ROS_INFO("(svis_ros) filtered time_offset_vec.size(): %lu", time_offset_vec_.size());

      // sum time offsets
      double sum = 0.0;
      for (int i = 0; i < time_offset_vec_.size(); i++) {
        sum += time_offset_vec_[i];
      }

      // calculate final time offset
      time_offset_ = sum / static_cast<double>(time_offset_vec_.size());
      ROS_INFO("(svis_ros) time_offset: %f", time_offset_);

      init_flag_ = false;
    }

    // check if we already sent a pulse and we have waiting long enough
    if (sent_pulse_) {
      // bail if we haven't waited long enough
      if ((ros::Time::now() - t_pulse_).toSec() < 0.5) {
        return;
      }

      // check strobe_buffer size
      if (strobe_buffer_.size() > 0 || camera_buffer_.size() > 0) {
        // we have exactly one of each
        if (strobe_buffer_.size() == 1 && camera_buffer_.size() == 1) {
          StrobePacket strobe = strobe_buffer_.front();
          CameraPacket camera = camera_buffer_.front();
          time_offset_vec_.push_back(camera.image.header.stamp.toSec() - strobe.timestamp_teensy);
          strobe_count_offset_ = camera.metadata.frame_counter - strobe.count_total;
          ROS_INFO("strobe_count_offset: %i", strobe_count_offset_);

          strobe_buffer_.pop_front();
          camera_buffer_.pop_front();
        } else {
          ROS_WARN("Mismatched strobe and camera buffer sizes");
          ROS_WARN("strobe_buffer size: %lu", strobe_buffer_.size());
          ROS_WARN("camera_buffer size: %lu", camera_buffer_.size());
          // clear buffers to reset counts
          strobe_buffer_.clear();
          camera_buffer_.clear();
        }

        sent_pulse_ = false;
      }
    } else {
      // send pulse if we haven't already
      SendPulse();
    }
  }

void SVISRos::GetHeader(std::vector<char> &buf, HeaderPacket &header) {
  tic();

  int ind = 0;

  // ros time
  header.timestamp_ros_rx = ros::Time::now().toSec();

  // send_count
  memcpy(&header.send_count, &buf[ind], sizeof(header.send_count));
  // ROS_INFO("(svis_ros) send_count: [%i, %i]", ind, header.send_count);
  ind += sizeof(header.send_count);

  // imu_packet_count
  memcpy(&header.imu_count, &buf[ind], sizeof(header.imu_count));
  // ROS_INFO("(svis_ros) imu_packet_count: [%i, %i]", ind, header.imu_count);
  ind += sizeof(header.imu_count);

  // strobe_packet_count
  memcpy(&header.strobe_count, &buf[ind], sizeof(header.strobe_count));
  // ROS_INFO("(svis_ros) strobe_packet_count: [%i, %i]", ind, header.strobe_count);
  ind += sizeof(header.strobe_count);

  timing_.get_header = toc();
}

void SVISRos::GetImu(std::vector<char> &buf, HeaderPacket &header, std::vector<ImuPacket> &imu_packets) {
  tic();

  for (int i = 0; i < header.imu_count; i++) {
    ImuPacket imu;
    int ind = imu_index[i];

    // ros receive time
    imu.timestamp_ros_rx = header.timestamp_ros_rx;

    // raw teensy timestamp
    memcpy(&imu.timestamp_teensy_raw, &buf[ind], sizeof(imu.timestamp_teensy_raw));
    // ROS_INFO("(svis_ros) imu.timestamp: [%i, %i]", ind, imu.timestamp);
    ind += sizeof(imu.timestamp_teensy_raw);

    // convert to seconds
    imu.timestamp_teensy = static_cast<double>(imu.timestamp_teensy_raw) / 1000000.0;

    // teensy time in ros epoch
    if (init_flag_) {
      imu.timestamp_ros = 0.0;
    } else {
      imu.timestamp_ros = imu.timestamp_teensy + time_offset_;
    }

    // accel
    memcpy(&imu.acc_raw[0], &buf[ind], sizeof(imu.acc_raw[0]));
    ind += sizeof(imu.acc_raw[0]);
    memcpy(&imu.acc_raw[1], &buf[ind], sizeof(imu.acc_raw[1]));
    ind += sizeof(imu.acc_raw[1]);
    memcpy(&imu.acc_raw[2], &buf[ind], sizeof(imu.acc_raw[2]));
    ind += sizeof(imu.acc_raw[2]);
    // ROS_INFO("(svis_ros) imu.acc_raw: [%i, %i, %i]", imu.acc_raw[0], imu.acc_raw[1], imu.acc_raw[2]);

    // convert accel
    imu.acc[0] = static_cast<float>(imu.acc_raw[0]) / acc_sens_arr_[acc_sens_] * g_;
    imu.acc[1] = static_cast<float>(imu.acc_raw[1]) / acc_sens_arr_[acc_sens_] * g_;
    imu.acc[2] = static_cast<float>(imu.acc_raw[2]) / acc_sens_arr_[acc_sens_] * g_;
    // ROS_INFO("(svis_ros) imu.acc: [%0.2f, %0.2f, %0.2f]", imu.acc[0], imu.acc[1], imu.acc[2]);

    // gyro
    memcpy(&imu.gyro_raw[0], &buf[ind], sizeof(imu.gyro_raw[0]));
    ind += sizeof(imu.gyro_raw[0]);
    memcpy(&imu.gyro_raw[1], &buf[ind], sizeof(imu.gyro_raw[1]));
    ind += sizeof(imu.gyro_raw[1]);
    memcpy(&imu.gyro_raw[2], &buf[ind], sizeof(imu.gyro_raw[2]));
    ind += sizeof(imu.gyro_raw[2]);
    // ROS_INFO("(svis_ros) imu.gyro_raw: [%i, %i, %i]", imu.gyro_raw[0], imu.gyro_raw[1], imu.gyro_raw[2]);

    // convert gyro
    imu.gyro[0] = static_cast<float>(imu.gyro_raw[0]) / gyro_sens_arr_[gyro_sens_] * rad_per_deg_;
    imu.gyro[1] = static_cast<float>(imu.gyro_raw[1]) / gyro_sens_arr_[gyro_sens_] * rad_per_deg_;
    imu.gyro[2] = static_cast<float>(imu.gyro_raw[2]) / gyro_sens_arr_[gyro_sens_] * rad_per_deg_;
    // ROS_INFO("(svis_ros) imu.gyro: [%0.2f, %0.2f, %0.2f]", imu.gyro[0], imu.gyro[1], imu.gyro[2]);

    // save packet
    imu_packets.push_back(imu);
  }

  timing_.get_imu = toc();
}

void SVISRos::GetStrobe(std::vector<char> &buf, HeaderPacket &header,
                 std::vector<StrobePacket> &strobe_packets) {
  tic();

  for (int i = 0; i < header.strobe_count; i++) {
    StrobePacket strobe;
    int ind = strobe_index[i];

    // ros time
    strobe.timestamp_ros_rx = header.timestamp_ros_rx;

    // timestamp
    memcpy(&strobe.timestamp_teensy_raw, &buf[ind], sizeof(strobe.timestamp_teensy_raw));
    // ROS_INFO("(svis_ros) strobe.timestamp: [%i, %i]", ind, strobe.timestamp);
    ind += sizeof(strobe.timestamp_teensy_raw);

    // convert to seconds
    strobe.timestamp_teensy = static_cast<double>(strobe.timestamp_teensy_raw) / 1000000.0;

    // teensy time in ros epoch
    if (init_flag_) {
      strobe.timestamp_ros = 0.0;
    } else {
      strobe.timestamp_ros = strobe.timestamp_teensy + time_offset_;
    }

    // count
    memcpy(&strobe.count, &buf[ind], sizeof(strobe.count));
    // ROS_INFO("(svis_ros) strobe.count: [%i, %i]", ind, strobe.count);
    ind += strobe.count;

    // save packet
    strobe_packets.push_back(strobe);
  }

  timing_.get_strobe = toc();
}

void SVISRos::PushImu(std::vector<ImuPacket> &imu_packets) {
  tic();

  ImuPacket imu;
  for (int i = 0; i < imu_packets.size(); i++) {
    imu = imu_packets[i];
    imu_buffer_.push_back(imu);
  }

  // warn if buffer is at max size
  if (imu_buffer_.size() == imu_buffer_.max_size()) {
    ROS_WARN("(svis_ros) imu buffer at max size");
  }

  timing_.push_imu = toc();
}

void SVISRos::PushStrobe(std::vector<StrobePacket> &strobe_packets) {
  tic();

  StrobePacket strobe;
  for (int i = 0; i < strobe_packets.size(); i++) {
    strobe = strobe_packets[i];
    strobe_buffer_.push_back(strobe);
  }

  // warn if buffer is at max size
  if (strobe_buffer_.size() == strobe_buffer_.max_size()) {
    ROS_WARN("(svis_ros) strobe buffer at max size");
  }

  timing_.push_strobe = toc();
}

void SVISRos::FilterImu(std::vector<ImuPacket> &imu_packets_filt) {
  tic();

  // create filter packets
  while (imu_buffer_.size() >= imu_filter_size_) {
    // sum
    float timestamp_total = 0.0;
    float acc_total[3] = {0.0};
    float gyro_total[3] = {0.0};
    ImuPacket temp_packet;
    for (int i = 0; i < imu_filter_size_; i++) {
      temp_packet = imu_buffer_[0];
      imu_buffer_.pop_front();

      timestamp_total += static_cast<double>(temp_packet.timestamp_teensy);
      for (int j = 0; j < 3; j++) {
        acc_total[j] += temp_packet.acc[j];
        gyro_total[j] += temp_packet.gyro[j];
      }
    }

    // calculate average (add 0.5 for rounding)
    temp_packet.timestamp_teensy =
      static_cast<int>(timestamp_total / static_cast<float>(imu_filter_size_) + 0.5);
    for (int j = 0; j < 3; j++) {
      temp_packet.acc[j] = acc_total[j] / static_cast<float>(imu_filter_size_);
      temp_packet.gyro[j] = gyro_total[j] / static_cast<float>(imu_filter_size_);
    }

    // save packet
    imu_packets_filt.push_back(temp_packet);
  }

  timing_.filter_imu = toc();
}

void SVISRos::PublishImu(std::vector<ImuPacket> &imu_packets_filt) {
  tic();

  ImuPacket temp_packet;
  sensor_msgs::Imu imu;

  for (int i = 0; i < imu_packets_filt.size(); i++) {
    temp_packet = imu_packets_filt[i];

    imu.header.stamp = ros::Time(temp_packet.timestamp_teensy + time_offset_);
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

  timing_.publish_imu = toc();
}

void SVISRos::PrintBuffer(std::vector<char> &buf) {
  ROS_INFO("(svis_ros) buffer: ");
  for (int i = 0; i < buf.size(); i++) {
    printf("%02X ", buf[i] & 255);
  }
  printf("\n");
}

void SVISRos::PrintImageQuadlet(std::string name, const sensor_msgs::Image::ConstPtr& msg, int i) {
  printf("%s: ", name.c_str());
  printf("%02X ", msg->data[i]);
  printf("%02X ", msg->data[i + 1]);
  printf("%02X ", msg->data[i + 2]);
  printf("%02X ", msg->data[i + 3]);
  printf("\n");
}

void SVISRos::PrintMetaDataRaw(const sensor_msgs::Image::ConstPtr& msg) {
  ROS_INFO("encoding: %s", msg->encoding.c_str());
  ROS_INFO("step: %i", msg->step);
  ROS_INFO("width: %i", msg->width);
  ROS_INFO("height: %i", msg->height);
  ROS_INFO("is_bigendian: %i", msg->is_bigendian);
  PrintImageQuadlet("timestamp", msg, 0);
  PrintImageQuadlet("gain", msg, 4);
  PrintImageQuadlet("shutter", msg, 8);
  PrintImageQuadlet("brightness", msg, 12);
  PrintImageQuadlet("exposure", msg, 16);
  PrintImageQuadlet("white balance", msg, 20);
  PrintImageQuadlet("frame counter", msg, 24);
  PrintImageQuadlet("roi", msg, 28);
  printf("\n\n");
}

void SVISRos::GetImageMetadata(const sensor_msgs::Image::ConstPtr& image_msg,
                               CameraPacket &camera_packet) {
  // timestamp
  memcpy(&camera_packet.metadata.timestamp, &image_msg->data[0],
         sizeof(camera_packet.metadata.timestamp));

  // gain
  memcpy(&camera_packet.metadata.gain, &image_msg->data[4],
         sizeof(camera_packet.metadata.gain));

  // shutter
  memcpy(&camera_packet.metadata.shutter, &image_msg->data[8],
         sizeof(camera_packet.metadata.shutter));

  // brightness
  memcpy(&camera_packet.metadata.brightness, &image_msg->data[12],
         sizeof(camera_packet.metadata.brightness));

  // exposure
  memcpy(&camera_packet.metadata.exposure, &image_msg->data[16],
         sizeof(camera_packet.metadata.exposure));

  // white balance
  memcpy(&camera_packet.metadata.white_balance, &image_msg->data[20],
         sizeof(camera_packet.metadata.white_balance));

  // frame counter
  uint32_t frame_counter = 0xFF & image_msg->data[27];
  frame_counter |= (0xFF & image_msg->data[26]) << 8;
  frame_counter |= (0xFF & image_msg->data[25]) << 16;
  frame_counter |= (0xFF & image_msg->data[24]) << 24;
  camera_packet.metadata.frame_counter = frame_counter;

  // region of interest
  memcpy(&camera_packet.metadata.roi_position, &image_msg->data[28],
         sizeof(camera_packet.metadata.roi_position));
}

void SVISRos::CameraCallback(const sensor_msgs::Image::ConstPtr& image_msg,
                             const sensor_msgs::CameraInfo::ConstPtr& info_msg) {
  if (!received_camera_) {
    received_camera_ = true;
  }

  // PrintMetaDataRaw(image_msg);
  CameraPacket camera_packet;

  // metadata
  GetImageMetadata(image_msg, camera_packet);
  // ROS_INFO("frame_count: %u", camera_packet.metadata.frame_counter);

  // set image and info
  camera_packet.image = *image_msg;
  camera_packet.info = *info_msg;

  // add to buffer
  camera_buffer_.push_back(camera_packet);

  // warn if buffer is at max size
  if (camera_buffer_.size() == camera_buffer_.max_size() && !sync_flag_) {
    ROS_WARN("(svis_ros) camera buffer at max size");
  }
}

void SVISRos::GetStrobeTotal(std::vector<StrobePacket> &strobe_packets) {
  tic();

  for (int i = 0; i < strobe_packets.size(); i++) {
    // ROS_INFO("strobe_count_total: %u", strobe_count_total_);

    // initialize variables on first iteration
    if (strobe_count_total_ == 0 && strobe_count_last_ == 0) {
      // ROS_INFO("init strobe count");
      strobe_count_total_ = 1;
      strobe_count_last_ = strobe_packets[i].count_total;
      strobe_packets[i].count_total = strobe_count_total_;
      continue;
    }

    // get count difference between two most recent strobe messages
    uint8_t diff = 0;
    if (strobe_packets[i].count > strobe_count_last_) {
      // no rollover
      diff = strobe_packets[i].count - strobe_count_last_;
    } else if (strobe_packets[i].count < strobe_count_last_) {
      // rollover
      diff = (strobe_count_last_ + strobe_packets[i].count);

      // handle rollover
      if (diff == 255) {
        // ROS_WARN("(svis_ros) Handle rollover");
        diff = 1;
      }
    } else {
      // no change
      ROS_WARN("(svis_ros) no change in strobe count");
    }

    // check diff value
    if (diff > 1 && !std::isinf(strobe_count_last_) && !init_flag_) {
      ROS_WARN("(svis_ros) detected jump in strobe count");
      // ROS_WARN("(svis_ros) diff: %i, last: %i, count: %i",
      //              diff,
      //              strobe_count_last_,
      //              strobe_packets[i].count);
    } else if (diff < 1 && !std::isinf(strobe_count_last_)) {
      ROS_WARN("(svis_ros) detected lag in strobe count");
    }

    // update count
    strobe_count_total_ += diff;

    // set packet total
    strobe_packets[i].count_total = strobe_count_total_;

    // update last value
    strobe_count_last_ = strobe_packets[i].count;
  }

  timing_.get_strobe_total = toc();
}

void SVISRos::GetCountOffset() {
  tic();

  std::vector<int> ind_vec(strobe_buffer_.size());
  std::vector<double> time_diff_vec(strobe_buffer_.size(),
                                    std::numeric_limits<double>::infinity());
  double time_diff = 0.0;

  // ROS_WARN("camera_buffer_size: %lu", camera_buffer_.size());
  // ROS_WARN("strobe_buffer_size: %lu", strobe_buffer_.size());

  // calculate time difference between images and strobes with corrected timestamps
  for (int i = 0; i < strobe_buffer_.size(); i++) {
    for (int j = 0; j < camera_buffer_.size(); j++) {
      time_diff = fabs(strobe_buffer_[i].timestamp_ros
                       - camera_buffer_[j].image.header.stamp.toSec());
      if (time_diff < time_diff_vec[i]) {
        time_diff_vec[i] = time_diff;
        ind_vec[i] = j;
      }
    }
  }

  // find min time diff
  double time_diff_best = std::numeric_limits<double>::infinity();
  int ind_best = 0;
  double time_diff_sum = 0.0;
  for (int i = 0; i < ind_vec.size(); i++) {
    time_diff_sum += time_diff_vec[i];
    if (time_diff_vec[i] < time_diff_best) {
      time_diff_best = time_diff_vec[i];
      ind_best = i;
    }
  }

  // calculate mean time difference for thresholding on quality of match
  double time_diff_mean = time_diff_sum / static_cast<double>(time_diff_vec.size());

  // print vector for debug
  // fprintf(stderr, "[ind_vec, time_diff_vec]:");
  // for (int i = 0; i < ind_vec.size(); i++) {
  //   fprintf(stderr, " %i (%f)", ind_vec[i], time_diff_vec[i]);

  //   // mark min time pair
  //   if (i == ind_best) {
  //     fprintf(stderr, "*");
  //   }
  // }
  // fprintf(stderr, "\n");

  // check quality of match
  // TODO(jakeware): don't hardcode rate
  if (time_diff_mean < 1.0/30.0) {
    sync_flag_ = false;

    // print counts
    // fprintf(stderr, "associated counts:\n");
    // for (int i = 0; i < strobe_buffer_.size(); i++) {
    //   fprintf(stderr, "[%i, %i] ",
    //           camera_buffer_[ind_vec[i]].metadata.frame_counter,
    //           strobe_buffer_[i].count_total);
    // }
    // fprintf(stderr, "\n");

    // print times
    // fprintf(stderr, "associated times:\n");
    // for (int i = 0; i < strobe_buffer_.size(); i++) {
    //   fprintf(stderr, "%f ", strobe_buffer_[i].timestamp_ros);
    // }
    // fprintf(stderr, "\n");

    // calculate offset
    strobe_count_offset_ = camera_buffer_[ind_vec[ind_best]].metadata.frame_counter
      - strobe_buffer_[ind_best].count_total;
    ROS_INFO("(svis_ros) strobe_count_offset: %i", strobe_count_offset_);
  }

  timing_.get_count_offset = toc();
}

void SVISRos::AssociateStrobe(std::vector<CameraStrobePacket> &camera_strobe_packets) {
  tic();

  // create camera strobe packets
  CameraStrobePacket camera_strobe;
  int fail_count = 0;
  int match_count = 0;
  bool match = false;

  // ROS_WARN("strobe_buffer size: %lu", strobe_buffer_.size());
  // ROS_WARN("camera_buffer size: %lu", camera_buffer_.size());

  for (auto it_strobe = strobe_buffer_.begin(); it_strobe != strobe_buffer_.end(); ) {
    // ROS_INFO("i: %lu", std::distance(strobe_buffer_.begin(), it_strobe));
    match = false;
    for (auto it_camera = camera_buffer_.begin(); it_camera != camera_buffer_.end(); ) {
      // ROS_INFO("j: %lu", std::distance(camera_buffer_.begin(), it_camera));
      // check for strobe/camera match
      if ((*it_strobe).count_total + strobe_count_offset_ ==
          (*it_camera).metadata.frame_counter) {
        // copy matched elements
        camera_strobe.camera = *it_camera;
        camera_strobe.strobe = *it_strobe;

        // fix timestamps
        // TODO(jakeware) fix issues with const here!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // camera_strobe.camera.info.header.stamp = ros::Time(camera_strobe.strobe.timestamp_ros);
        // camera_strobe.camera.image.header.stamp = ros::Time(camera_strobe.strobe.timestamp_ros);

        // push to buffer
        camera_strobe_packets.push_back(camera_strobe);

        // remove matched strobe
        // ROS_INFO("delete matched camera");
        it_camera = camera_buffer_.erase(it_camera);

        // record match
        match_count++;
        match = true;
        // ROS_INFO("match");
        break;
      } else {
        // check for stale entry and delete
        if ((ros::Time::now().toSec() - (*it_camera).image.header.stamp.toSec()) > 1.0) {
          // ROS_INFO("delete stale camera");
          it_camera = camera_buffer_.erase(it_camera);
        } else {
          // ROS_INFO("increment camera");
          ++it_camera;
        }
      }
    }

    // increment fail count for no images for a given strobe message
    if (match) {
      // remove matched strobe
      // ROS_INFO("delete matched strobe");
      it_strobe = strobe_buffer_.erase(it_strobe);
    } else {
      fail_count++;
      // ROS_INFO("fail");

      // check for stale entry and delete
      if ((ros::Time::now().toSec() - (*it_strobe).timestamp_ros_rx) > 1.0) {
        ROS_WARN("(svis ros) Delete stale strobe");
        it_strobe = strobe_buffer_.erase(it_strobe);
      } else {
        // ROS_INFO("increment strobe");
        ++it_strobe;
      }
    }
  }

  // ROS_INFO("final fail_count: %i", fail_count);
  // ROS_INFO("final match_count: %i", match_count);
  if (fail_count == strobe_buffer_.max_size()) {
    ROS_WARN("Failure to match.  Resyncing...");
    sync_flag_ = true;
  }

  // // erase camera images that are older than all strobes
  // bool stale = true;
  // for (auto it_camera = camera_buffer_.begin(); it_camera != camera_buffer_.end(); ) {
  //   stale = true;
  //   for (auto it_strobe = strobe_buffer_.begin(); it_strobe != strobe_buffer_.end(); ++it_strobe) {
  //     // check if strobe is older than camera
  //     if ((*it_strobe).count_total + strobe_count_offset_ <=
  //         (*it_camera).metadata.frame_counter) {
  //       stale = false;
  //     }
  //   }

  //   // remove camera if stale
  //   if (stale) {
  //     it_camera = camera_buffer_.erase(it_camera);
  //   } else {
  //     ++it_camera;
  //   }
  // }

  timing_.associate_strobe = toc();
}

void SVISRos::PublishCamera(std::vector<CameraStrobePacket> &camera_strobe_packets) {
  tic();

  for (int i = 0; i < camera_strobe_packets.size(); i++) {
    camera_pub_.publish(camera_strobe_packets[i].camera.image,
                        camera_strobe_packets[i].camera.info, ros::Time(camera_strobe_packets[i].strobe.timestamp_ros));
  }

  camera_strobe_packets.clear();

  timing_.publish_camera = toc();
}

void SVISRos::PublishImuRaw(std::vector<ImuPacket> &imu_packets) {
  tic();

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

  timing_.publish_imu_raw = toc();
}

void SVISRos::PublishStrobeRaw(std::vector<StrobePacket> &strobe_packets) {
  tic();

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

  timing_.publish_strobe_raw = toc();
}

void SVISRos::PrintCameraBuffer() {
  double t_now = ros::Time::now().toSec();
  printf("camera_buffer: %lu\n", camera_buffer_.size());
  for (int i = 0; i < camera_buffer_.size(); i++) {
    printf("%i:(%i)%f ", i, camera_buffer_[i].metadata.frame_counter, t_now - camera_buffer_[i].image.header.stamp.toSec());
  }
  printf("\n\n");
}

void SVISRos::PrintStrobeBuffer() {
  double t_now = ros::Time::now().toSec();
  printf("strobe_buffer: %lu\n", strobe_buffer_.size());
  for (int i = 0; i < strobe_buffer_.size(); i++) {
    printf("%i:(%i, %i)%f ", i, strobe_buffer_[i].count, strobe_buffer_[i].count_total + strobe_count_offset_, t_now - strobe_buffer_[i].timestamp_ros);
  }
  printf("\n");
}

void SVISRos::tic() {
  tic_ = ros::Time::now();
}

double SVISRos::toc() {
  toc_ = ros::Time::now();

  return (toc_ - tic_).toSec();
}

void SVISRos::PublishTiming() {
  svis_timing_pub_.publish(timing_);
}

}  // namespace svis_ros
