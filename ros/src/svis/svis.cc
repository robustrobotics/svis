// Copyright 2017 Massachusetts Institute of Technology

#include "svis/svis.h"

namespace svis {

void SVIS::InitHID() {
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
    printf("(svis_ros) No svis_teensy device found.\n");
    return;
  } else {
    printf("(svis_ros) Found svis_teensy device\n");
  }
}

bool SVIS::ReadHID(std::vector<char>* buf) {
  // check if any Raw HID packet has arrived
  int num = rawhid_recv(0, buf->data(), buf->size(), 220);

  // check byte count
  if (num < 0) {
    printf("(svis_ros): Error reading, device went offline");
    rawhid_close(0);
    exit(1);
  } else if (num == 0) {
    if (!init_flag_) {
      printf("(svis_ros) 0 bytes received");
    }
  } else if (num > 0) {
    // resize vector
    buf->resize(num);

    // debug print
    if (print_buffer_) {
      PrintBuffer(*buf);
    }
  } else {
    printf("(svis_ros) Bad return value from rawhid_recv");
  }

  return GetChecksum(*buf);
}

void SVIS::Update() {

  //   // read hid serial
  //   std::vector<char> buf_(64, 0);
  //   if (svis.ReadHID(&buf)) {
  //     continue;  // bail if checksums don't match
  //   }

  //   // parse
  //   std::vector<ImuPacket> imu_packets;
  //   std::vector<StrobePacket> strobe_packets;
  //   svis.ParseHID(buf, &imu_packets, &strobe_packets);

  //   // publish raw packets
  //   PublishImuRaw(imu_packets);
  //   PublishStrobeRaw(strobe_packets);

  //   // get difference between ros and teensy epochs
  //   if (svis.init_flag_) {
  //     svis.GetTimeOffset();
  //     continue;
  //   }

  //   // filter and publish imu
  //   std::vector<ImuPacket> imu_packets_filt;

  //   tic();
  //   svis.FilterImu(imu_packets_filt);
  //   timing_.filter_imu = toc();
    
  //   PublishImu(imu_packets_filt);

  //   // associate strobe with camera and publish
  //   std::vector<CameraStrobePacket> camera_strobe_packets;
  //   svis.AssociateStrobe(camera_strobe_packets);
  //   PublishCamera(camera_strobe_packets);
}

void SVIS::ParseHID(const std::vector<char>& buf, std::vector<ImuPacket>* imu_packets, std::vector<StrobePacket>* strobe_packets) {
  // parse header
  HeaderPacket header;
  // GetHeader(buf, header);

  // parse, publish, push imu
  // GetImu(buf, header, imu_packets);
  // PushImu(imu_packets);

  // parse, publish, push strobe
  // GetStrobe(buf, header, strobe_packets);
  // GetStrobeTotal(strobe_packets);
  // PushStrobe(strobe_packets);
}

void SVIS::SendPulse() {
  std::vector<char> buf(64, 0);
  buf[0] = 0xAB;
  buf[1] = 2;
  printf("(svis_ros) Sending pulse packet");
  rawhid_send(0, buf.data(), buf.size(), 100);
  sent_pulse_ = true;
  t_pulse_ = ros::Time::now();
}

void SVIS::SendDisablePulse() {
  std::vector<char> buf(64, 0);
  buf[0] = 0xAB;
  buf[1] = 3;
  printf("(svis_ros) Sending configuration packet");
  rawhid_send(0, buf.data(), buf.size(), 100);
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

  printf("(svis_ros) Sending configuration packet");
  rawhid_send(0, buf.data(), buf.size(), 100);
}

int SVIS::GetChecksum(const std::vector<char>& buf) {
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
    printf("(svis_ros) checksum error [%02X, %02X] [%02X, %02X]",
                 buf[checksum_index] & 0xFF, buf[checksum_index + 1] & 0xFF, checksum_calc, checksum_orig);
  }

  return ret;
}

void SVIS::GetTimeOffset() {
  if (time_offset_vec_.size() >= 100) {
    // turn off camera pulse
    SendDisablePulse();

    // filter initial values that are often composed of stale data
    // printf("(svis_ros) time_offset_vec.size(): %lu", time_offset_vec_.size());
    while (fabs(time_offset_vec_.front() - time_offset_vec_.back()) > 0.1) {
      time_offset_vec_.pop_front();
    }
    // printf("(svis_ros) filtered time_offset_vec.size(): %lu", time_offset_vec_.size());

    // sum time offsets
    double sum = 0.0;
    for (int i = 0; i < time_offset_vec_.size(); i++) {
      sum += time_offset_vec_[i];
    }

    // calculate final time offset
    time_offset_ = sum / static_cast<double>(time_offset_vec_.size());
    printf("(svis_ros) time_offset: %f", time_offset_);

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
        printf("strobe_count_offset: %i", strobe_count_offset_);

        strobe_buffer_.pop_front();
        camera_buffer_.pop_front();
      } else {
        printf("Mismatched strobe and camera buffer sizes");
        printf("strobe_buffer size: %lu", strobe_buffer_.size());
        printf("camera_buffer size: %lu", camera_buffer_.size());
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

void SVIS::GetHeader(const std::vector<char>& buf, HeaderPacket &header) {
  int ind = 0;

  // ros time
  header.timestamp_ros_rx = ros::Time::now().toSec();

  // send_count
  memcpy(&header.send_count, &buf[ind], sizeof(header.send_count));
  // printf("(svis_ros) send_count: [%i, %i]", ind, header.send_count);
  ind += sizeof(header.send_count);

  // imu_packet_count
  memcpy(&header.imu_count, &buf[ind], sizeof(header.imu_count));
  // printf("(svis_ros) imu_packet_count: [%i, %i]", ind, header.imu_count);
  ind += sizeof(header.imu_count);

  // strobe_packet_count
  memcpy(&header.strobe_count, &buf[ind], sizeof(header.strobe_count));
  // printf("(svis_ros) strobe_packet_count: [%i, %i]", ind, header.strobe_count);
  ind += sizeof(header.strobe_count);
}

void SVIS::GetImu(const std::vector<char>& buf, HeaderPacket &header, std::vector<ImuPacket> &imu_packets) {
  for (int i = 0; i < header.imu_count; i++) {
    ImuPacket imu;
    int ind = imu_index[i];

    // ros receive time
    imu.timestamp_ros_rx = header.timestamp_ros_rx;

    // raw teensy timestamp
    memcpy(&imu.timestamp_teensy_raw, &buf[ind], sizeof(imu.timestamp_teensy_raw));
    // printf("(svis_ros) imu.timestamp: [%i, %i]", ind, imu.timestamp);
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
    // printf("(svis_ros) imu.acc_raw: [%i, %i, %i]", imu.acc_raw[0], imu.acc_raw[1], imu.acc_raw[2]);

    // convert accel
    imu.acc[0] = static_cast<float>(imu.acc_raw[0]) / acc_sens_arr_[acc_sens_] * g_;
    imu.acc[1] = static_cast<float>(imu.acc_raw[1]) / acc_sens_arr_[acc_sens_] * g_;
    imu.acc[2] = static_cast<float>(imu.acc_raw[2]) / acc_sens_arr_[acc_sens_] * g_;
    // printf("(svis_ros) imu.acc: [%0.2f, %0.2f, %0.2f]", imu.acc[0], imu.acc[1], imu.acc[2]);

    // gyro
    memcpy(&imu.gyro_raw[0], &buf[ind], sizeof(imu.gyro_raw[0]));
    ind += sizeof(imu.gyro_raw[0]);
    memcpy(&imu.gyro_raw[1], &buf[ind], sizeof(imu.gyro_raw[1]));
    ind += sizeof(imu.gyro_raw[1]);
    memcpy(&imu.gyro_raw[2], &buf[ind], sizeof(imu.gyro_raw[2]));
    ind += sizeof(imu.gyro_raw[2]);
    // printf("(svis_ros) imu.gyro_raw: [%i, %i, %i]", imu.gyro_raw[0], imu.gyro_raw[1], imu.gyro_raw[2]);

    // convert gyro
    imu.gyro[0] = static_cast<float>(imu.gyro_raw[0]) / gyro_sens_arr_[gyro_sens_] * rad_per_deg_;
    imu.gyro[1] = static_cast<float>(imu.gyro_raw[1]) / gyro_sens_arr_[gyro_sens_] * rad_per_deg_;
    imu.gyro[2] = static_cast<float>(imu.gyro_raw[2]) / gyro_sens_arr_[gyro_sens_] * rad_per_deg_;
    // printf("(svis_ros) imu.gyro: [%0.2f, %0.2f, %0.2f]", imu.gyro[0], imu.gyro[1], imu.gyro[2]);

    // save packet
    imu_packets.push_back(imu);
  }
}

void SVIS::GetStrobe(const std::vector<char>& buf, HeaderPacket &header,
                 std::vector<StrobePacket>& strobe_packets) {
  for (int i = 0; i < header.strobe_count; i++) {
    StrobePacket strobe;
    int ind = strobe_index[i];

    // ros time
    strobe.timestamp_ros_rx = header.timestamp_ros_rx;

    // timestamp
    memcpy(&strobe.timestamp_teensy_raw, &buf[ind], sizeof(strobe.timestamp_teensy_raw));
    // printf("(svis_ros) strobe.timestamp: [%i, %i]", ind, strobe.timestamp);
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
    // printf("(svis_ros) strobe.count: [%i, %i]", ind, strobe.count);
    ind += strobe.count;

    // save packet
    strobe_packets.push_back(strobe);
  }
}

void SVIS::PushImu(std::vector<ImuPacket>& imu_packets) {
  ImuPacket imu;
  for (int i = 0; i < imu_packets.size(); i++) {
    imu = imu_packets[i];
    imu_buffer_.push_back(imu);
  }

  // warn if buffer is at max size
  if (imu_buffer_.size() == imu_buffer_.max_size()) {
    printf("(svis_ros) imu buffer at max size");
  }
}

void SVIS::PushStrobe(std::vector<StrobePacket> &strobe_packets) {
  StrobePacket strobe;
  for (int i = 0; i < strobe_packets.size(); i++) {
    strobe = strobe_packets[i];
    strobe_buffer_.push_back(strobe);
  }

  // warn if buffer is at max size
  if (strobe_buffer_.size() == strobe_buffer_.max_size()) {
    printf("(svis_ros) strobe buffer at max size");
  }
}

void SVIS::FilterImu(std::vector<ImuPacket> &imu_packets_filt) {
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
}

void SVIS::PrintBuffer(const std::vector<char> &buf) {
  printf("(svis_ros) buffer: ");
  for (int i = 0; i < buf.size(); i++) {
    printf("%02X ", buf[i] & 255);
  }
  printf("\n");
}

void SVIS::GetStrobeTotal(std::vector<StrobePacket>* strobe_packets) {
  // for (int i = 0; i < strobe_packets->size(); i++) {
  //   // printf("strobe_count_total: %u", strobe_count_total_);

  //   // initialize variables on first iteration
  //   if (strobe_count_total_ == 0 && strobe_count_last_ == 0) {
  //     // printf("init strobe count");
  //     strobe_count_total_ = 1;
  //     strobe_count_last_ = strobe_packets[i]->count_total;
  //     strobe_packets[i]->count_total = strobe_count_total_;
  //     continue;
  //   }

  //   // get count difference between two most recent strobe messages
  //   uint8_t diff = 0;
  //   if (strobe_packets[i]->count > strobe_count_last_) {
  //     // no rollover
  //     diff = strobe_packets[i]->count - strobe_count_last_;
  //   } else if (strobe_packets[i]->count < strobe_count_last_) {
  //     // rollover
  //     diff = (strobe_count_last_ + strobe_packets[i]->count);

  //     // handle rollover
  //     if (diff == 255) {
  //       // printf("(svis_ros) Handle rollover");
  //       diff = 1;
  //     }
  //   } else {
  //     // no change
  //     printf("(svis_ros) no change in strobe count");
  //   }

  //   // check diff value
  //   if (diff > 1 && !std::isinf(strobe_count_last_) && !init_flag_) {
  //     printf("(svis_ros) detected jump in strobe count");
  //     // printf("(svis_ros) diff: %i, last: %i, count: %i",
  //     //              diff,
  //     //              strobe_count_last_,
  //     //              strobe_packets[i].count);
  //   } else if (diff < 1 && !std::isinf(strobe_count_last_)) {
  //     printf("(svis_ros) detected lag in strobe count");
  //   }

  //   // update count
  //   strobe_count_total_ += diff;

  //   // set packet total
  //   strobe_packets[i]->count_total = strobe_count_total_;

  //   // update last value
  //   strobe_count_last_ = strobe_packets[i]->count;
  // }
}

void SVIS::GetCountOffset() {
  std::vector<int> ind_vec(strobe_buffer_.size());
  std::vector<double> time_diff_vec(strobe_buffer_.size(),
                                    std::numeric_limits<double>::infinity());
  double time_diff = 0.0;

  // printf("camera_buffer_size: %lu", camera_buffer_.size());
  // printf("strobe_buffer_size: %lu", strobe_buffer_.size());

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
    printf("(svis_ros) strobe_count_offset: %i", strobe_count_offset_);
  }
}

void SVIS::AssociateStrobe(std::vector<CameraStrobePacket> &camera_strobe_packets) {
  // create camera strobe packets
  CameraStrobePacket camera_strobe;
  int fail_count = 0;
  int match_count = 0;
  bool match = false;

  // printf("strobe_buffer size: %lu", strobe_buffer_.size());
  // printf("camera_buffer size: %lu", camera_buffer_.size());

  for (auto it_strobe = strobe_buffer_.begin(); it_strobe != strobe_buffer_.end(); ) {
    // printf("i: %lu", std::distance(strobe_buffer_.begin(), it_strobe));
    match = false;
    for (auto it_camera = camera_buffer_.begin(); it_camera != camera_buffer_.end(); ) {
      // printf("j: %lu", std::distance(camera_buffer_.begin(), it_camera));
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
        // printf("delete matched camera");
        it_camera = camera_buffer_.erase(it_camera);

        // record match
        match_count++;
        match = true;
        // printf("match");
        break;
      } else {
        // check for stale entry and delete
        if ((ros::Time::now().toSec() - (*it_camera).image.header.stamp.toSec()) > 1.0) {
          // printf("delete stale camera");
          it_camera = camera_buffer_.erase(it_camera);
        } else {
          // printf("increment camera");
          ++it_camera;
        }
      }
    }

    // increment fail count for no images for a given strobe message
    if (match) {
      // remove matched strobe
      // printf("delete matched strobe");
      it_strobe = strobe_buffer_.erase(it_strobe);
    } else {
      fail_count++;
      // printf("fail");

      // check for stale entry and delete
      if ((ros::Time::now().toSec() - (*it_strobe).timestamp_ros_rx) > 1.0) {
        printf("(svis ros) Delete stale strobe");
        it_strobe = strobe_buffer_.erase(it_strobe);
      } else {
        // printf("increment strobe");
        ++it_strobe;
      }
    }
  }

  // printf("final fail_count: %i", fail_count);
  // printf("final match_count: %i", match_count);
  if (fail_count == strobe_buffer_.max_size()) {
    printf("Failure to match.  Resyncing...");
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
}

void SVIS::PrintCameraBuffer() {
  double t_now = ros::Time::now().toSec();
  printf("camera_buffer: %lu\n", camera_buffer_.size());
  for (int i = 0; i < camera_buffer_.size(); i++) {
    printf("%i:(%i)%f ", i, camera_buffer_[i].metadata.frame_counter, t_now - camera_buffer_[i].image.header.stamp.toSec());
  }
  printf("\n\n");
}

void SVIS::PrintStrobeBuffer() {
  double t_now = ros::Time::now().toSec();
  printf("strobe_buffer: %lu\n", strobe_buffer_.size());
  for (int i = 0; i < strobe_buffer_.size(); i++) {
    printf("%i:(%i, %i)%f ", i, strobe_buffer_[i].count, strobe_buffer_[i].count_total + strobe_count_offset_, t_now - strobe_buffer_[i].timestamp_ros);
  }
  printf("\n");
}

void SVIS::PrintImageQuadlet(std::string name, const sensor_msgs::Image::ConstPtr& msg, int i) {
  printf("%s: ", name.c_str());
  printf("%02X ", msg->data[i]);
  printf("%02X ", msg->data[i + 1]);
  printf("%02X ", msg->data[i + 2]);
  printf("%02X ", msg->data[i + 3]);
  printf("\n");
}

void SVIS::PrintMetaDataRaw(const sensor_msgs::Image::ConstPtr& msg) {
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

void SVIS::GetImageMetadata(const sensor_msgs::Image::ConstPtr& image_msg,
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

void SVIS::tic() {
  tic_ = ros::Time::now();
}

double SVIS::toc() {
  toc_ = ros::Time::now();

  return (toc_ - tic_).toSec();
}

}  // namespace svis
