// Copyright 2017 Massachusetts Institute of Technology

#include <cstring>

#include "svis/svis.h"

namespace svis {

SVIS::SVIS() {
  // set circular buffer max lengths
  imu_buffer_.set_capacity(30);
  strobe_buffer_.set_capacity(30);
  camera_buffer_.set_capacity(30);
}

double SVIS::GetTimeOffset() const {
  return time_offset_;
}

std::size_t SVIS::GetCameraBufferSize() const {
  return camera_buffer_.size();
}

std::size_t SVIS::GetCameraBufferMaxSize() const {
  return camera_buffer_.max_size();
}

bool SVIS::GetSyncFlag() const {
  return sync_flag_;
}

void SVIS::PushCameraPacket(const svis::CameraPacket& camera_packet) {
  camera_buffer_.push_back(camera_packet);
}

void SVIS::OpenHID() {
  // open rawhid port
  // C-based example is 16C0:0480:FFAB:0200
  // Arduino-based example is 16C0:0486:FFAB:0200
  int r = rawhid_open(1, 0x16C0, 0x0486, 0xFFAB, 0x0200);

  // check return
  if (r <= 0) {
    printf("(svis) No svis_teensy device found.\n");
    return;
  } else {
    printf("(svis) Found svis_teensy device\n");
  }
}

int SVIS::ReadHID(std::vector<char>* buf) {
  // check if any Raw HID packet has arrived
  tic();
  int num = rawhid_recv(0, buf->data(), buf->size(), 220);
  timing_.rawhid_recv = toc();

  // check byte count
  if (num < 0) {
    printf("(svis) Error: reading, device went offline\n");
    rawhid_close(0);
    exit(1);
  } else if (num == 0) {
    if (!init_flag_) {
      printf("(svis) 0 bytes received\n");
    }
  } else if (num > 0) {
    // resize vector
    buf->resize(num);
  } else {
    printf("(svis) Bad return value from rawhid_recv\n");
  }

  return num;
}

void SVIS::Update() {
  std::chrono::time_point<std::chrono::high_resolution_clock>
    t_update_start_ = std::chrono::high_resolution_clock::now();
  
  // read and return if empty or bad
  std::vector<char> buf(64, 0);
  if (ReadHID(&buf) <= 0) {
    return;
  }

  // debug print
  if (print_buffer_) {
    PrintBuffer(buf);
  }

  // check checksums and return if they don't match
  if (CheckChecksum(buf)) {
    return;
  }

  // parse packets
  std::vector<ImuPacket> imu_packets;
  std::vector<StrobePacket> strobe_packets;
  ParseBuffer(buf, &imu_packets, &strobe_packets);

  // handle strobe
  PushStrobe(strobe_packets, &strobe_buffer_);
  PublishStrobeRaw(strobe_packets);

  // get difference between ros and teensy epochs
  if (init_flag_) {
    ComputeOffsets(&strobe_buffer_, &camera_buffer_);
    return;
  }

  // handle imu
  PushImu(imu_packets, &imu_buffer_);
  PublishImuRaw(imu_packets);

  // filter and publish imu
  std::vector<ImuPacket> imu_packets_filt;
  FilterImu(&imu_buffer_, &imu_packets_filt);
  // DecimateImu(&imu_buffer_, &imu_packets_filt);
  for (int i = 0; i < imu_packets_filt.size(); i++) {
    usleep(500);
    PublishImu(imu_packets_filt[i]);
  }

  // associate strobe with camera and publish
  std::vector<CameraStrobePacket> camera_strobe_packets;
  Associate(&strobe_buffer_, &camera_buffer_, &camera_strobe_packets);
  PublishCamera(camera_strobe_packets);
  camera_strobe_packets.clear();

  std::chrono::duration<double> update_duration = std::chrono::high_resolution_clock::now() - t_update_start_;
  timing_.update = update_duration.count();
  PublishTiming(timing_);
  timing_ = svis::Timing(); // clear timing
}

void SVIS::ParseBuffer(const std::vector<char>& buf, std::vector<ImuPacket>* imu_packets, std::vector<StrobePacket>* strobe_packets) {
  HeaderPacket header;
  ParseHeader(buf, &header);
  ParseImu(buf, header, imu_packets);
  ParseStrobe(buf, header, strobe_packets);
  ComputeStrobeTotal(strobe_packets);
}

void SVIS::SendPulse() {
  std::vector<char> buf(64, 0);
  buf[0] = 0xAB;
  buf[1] = 2;
  printf("(svis) Sending pulse packet\n");
  rawhid_send(0, buf.data(), buf.size(), 100);
  sent_pulse_ = true;
  t_pulse_ = std::chrono::high_resolution_clock::now();
}

void SVIS::SendDisablePulse() {
  std::vector<char> buf(64, 0);
  buf[0] = 0xAB;
  buf[1] = 3;
  printf("(svis) Sending configuration packet\n");
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

  printf("(svis) Sending configuration packet\n");
  rawhid_send(0, buf.data(), buf.size(), 100);
}

bool SVIS::CheckChecksum(const std::vector<char>& buf) {
  tic();

  // calculate checksum
  uint16_t checksum_calc = 0;
  for (int i = 0; i < send_buffer_size - 2; i++) {
    checksum_calc += buf[i] & 0xFF;
  }

  // get packet chechsum
  uint16_t checksum_orig = 0;
  std::memcpy(&checksum_orig, &buf[checksum_index], sizeof(checksum_orig));
  bool ret = (checksum_calc != checksum_orig);

  // check match and return result
  if (ret) {
    printf("(svis) checksum error [%02X, %02X] [%02X, %02X]\n",
                 buf[checksum_index] & 0xFF, buf[checksum_index + 1] & 0xFF, checksum_calc, checksum_orig);
  }

  timing_.check_checksum = toc();

  return ret;
}

void SVIS::ComputeOffsets(boost::circular_buffer<StrobePacket>* strobe_buffer,
                         boost::circular_buffer<CameraPacket>* camera_buffer) {
  tic();

  if (time_offset_vec_.size() >= static_cast<std::size_t>(offset_sample_count_)) {
    // turn off camera pulse
    SendDisablePulse();

    // filter initial values that are often composed of stale data
    // printf("(svis) time_offset_vec.size(): %lu\n", time_offset_vec_.size());
    while (fabs(time_offset_vec_.front() - time_offset_vec_.back()) > 0.1) {
      time_offset_vec_.pop_front();
    }
    // printf("(svis) filtered time_offset_vec.size(): %lu\n", time_offset_vec_.size());

    // sum time offsets
    double sum = 0.0;
    for (uint i = 0; i < time_offset_vec_.size(); i++) {
      sum += time_offset_vec_[i];
    }

    // calculate final time offset
    time_offset_ = sum / static_cast<double>(time_offset_vec_.size());
    printf("(svis) time_offset: %f\n", time_offset_);

    init_flag_ = false;
  }

  // check if we already sent a pulse and we have waiting long enough
  if (sent_pulse_) {
    // bail if we haven't waited long enough
    std::chrono::duration<double> pulse_duration = std::chrono::high_resolution_clock::now() - t_pulse_;
    if (pulse_duration.count() < offset_sample_time_) {
      return;
    }

    // check strobe_buffer size
    if (strobe_buffer->size() > 0 || camera_buffer->size() > 0) {
      // we have exactly one of each
      if (strobe_buffer->size() == 1 && camera_buffer->size() == 1) {
        StrobePacket strobe = strobe_buffer->front();
        CameraPacket camera = camera_buffer->front();
        time_offset_vec_.push_back(camera.image.header.stamp - strobe.timestamp_teensy);
        strobe_count_offset_ = camera.metadata.frame_counter - strobe.count_total;
        printf("strobe_count_offset: %i\n", strobe_count_offset_);

        strobe_buffer->pop_front();
        camera_buffer->pop_front();
      } else {
        printf("Mismatched strobe and camera buffer sizes\n");
        printf("strobe_buffer size: %lu\n", strobe_buffer->size());
        printf("camera_buffer size: %lu\n", camera_buffer->size());
        // clear buffers to reset counts
        strobe_buffer->clear();
        camera_buffer->clear();
      }

      sent_pulse_ = false;
    }
  } else {
    // send pulse if we haven't already
    SendPulse();
  }

  timing_.compute_offsets = toc();
}

void SVIS::ParseHeader(const std::vector<char>& buf, HeaderPacket* header) {
  tic();

  int ind = 0;

  // ros time
  header->timestamp_ros_rx = TimeNow();

  // send_count
  memcpy(&header->send_count, &buf[ind], sizeof(header->send_count));
  // printf("(svis) send_count: [%i, %i]\n", ind, header->send_count);
  ind += sizeof(header->send_count);

  // imu_packet_count
  memcpy(&header->imu_count, &buf[ind], sizeof(header->imu_count));
  // printf("(svis) imu_packet_count: [%i, %i]\n", ind, header->imu_count);
  ind += sizeof(header->imu_count);

  // strobe_packet_count
  memcpy(&header->strobe_count, &buf[ind], sizeof(header->strobe_count));
  // printf("(svis) strobe_packet_count: [%i, %i]\n", ind, header->strobe_count);
  ind += sizeof(header->strobe_count);

  timing_.parse_header = toc();
}

void SVIS::ParseImu(const std::vector<char>& buf, const HeaderPacket& header, std::vector<ImuPacket>* imu_packets) {
  tic();
  for (int i = 0; i < header.imu_count; i++) {
    ImuPacket imu;
    int ind = imu_index[i];

    // ros receive time
    imu.timestamp_ros_rx = header.timestamp_ros_rx;

    // raw teensy timestamp
    memcpy(&imu.timestamp_teensy_raw, &buf[ind], sizeof(imu.timestamp_teensy_raw));
    ind += sizeof(imu.timestamp_teensy_raw);

    // convert to seconds
    imu.timestamp_teensy = static_cast<double>(imu.timestamp_teensy_raw) / 1000000.0;

    // teensy time in ros epoch
    if (init_flag_) {
      imu.timestamp_ros = 0.0;
    } else {
      imu.timestamp_ros = imu.timestamp_teensy + GetTimeOffset();
    }
    
    // accel
    memcpy(&imu.acc_raw[0], &buf[ind], sizeof(imu.acc_raw[0]));
    ind += sizeof(imu.acc_raw[0]);
    memcpy(&imu.acc_raw[1], &buf[ind], sizeof(imu.acc_raw[1]));
    ind += sizeof(imu.acc_raw[1]);
    memcpy(&imu.acc_raw[2], &buf[ind], sizeof(imu.acc_raw[2]));
    ind += sizeof(imu.acc_raw[2]);
    // printf("(svis) imu.acc_raw: [%i, %i, %i]\n", imu.acc_raw[0], imu.acc_raw[1], imu.acc_raw[2]);

    // convert accel
    imu.acc[0] = static_cast<float>(imu.acc_raw[0]) / acc_sens_arr_[acc_sens_] * g_;
    imu.acc[1] = static_cast<float>(imu.acc_raw[1]) / acc_sens_arr_[acc_sens_] * g_;
    imu.acc[2] = static_cast<float>(imu.acc_raw[2]) / acc_sens_arr_[acc_sens_] * g_;
    // printf("(svis) imu.acc: [%0.2f, %0.2f, %0.2f]\n", imu.acc[0], imu.acc[1], imu.acc[2]);

    // gyro
    memcpy(&imu.gyro_raw[0], &buf[ind], sizeof(imu.gyro_raw[0]));
    ind += sizeof(imu.gyro_raw[0]);
    memcpy(&imu.gyro_raw[1], &buf[ind], sizeof(imu.gyro_raw[1]));
    ind += sizeof(imu.gyro_raw[1]);
    memcpy(&imu.gyro_raw[2], &buf[ind], sizeof(imu.gyro_raw[2]));
    ind += sizeof(imu.gyro_raw[2]);
    // printf("(svis) imu.gyro_raw: [%i, %i, %i]\n", imu.gyro_raw[0], imu.gyro_raw[1], imu.gyro_raw[2]);

    // convert gyro
    imu.gyro[0] = static_cast<float>(imu.gyro_raw[0]) / gyro_sens_arr_[gyro_sens_] * rad_per_deg_;
    imu.gyro[1] = static_cast<float>(imu.gyro_raw[1]) / gyro_sens_arr_[gyro_sens_] * rad_per_deg_;
    imu.gyro[2] = static_cast<float>(imu.gyro_raw[2]) / gyro_sens_arr_[gyro_sens_] * rad_per_deg_;
    // printf("(svis) imu.gyro: [%0.2f, %0.2f, %0.2f]\n", imu.gyro[0], imu.gyro[1], imu.gyro[2]);

    // save packet
    // printf("(svis) imu.timestamp_ros: [%i, %f, %f]\n", ind, imu.timestamp_ros, imu.acc[2]);
    imu_packets->push_back(imu);

    timing_.parse_imu = toc();
  }
}

void SVIS::ParseStrobe(const std::vector<char>& buf,
                     const HeaderPacket& header,
                     std::vector<StrobePacket>* strobe_packets) {
  tic();

  for (uint i = 0; i < header.strobe_count; i++) {
    StrobePacket strobe;
    int ind = strobe_index[i];

    // ros time
    strobe.timestamp_ros_rx = header.timestamp_ros_rx;

    // timestamp
    memcpy(&strobe.timestamp_teensy_raw, &buf[ind], sizeof(strobe.timestamp_teensy_raw));
    // printf("(svis) strobe.timestamp: [%i, %i]\n", ind, strobe.timestamp);
    ind += sizeof(strobe.timestamp_teensy_raw);

    // convert to seconds
    strobe.timestamp_teensy = static_cast<double>(strobe.timestamp_teensy_raw) / 1000000.0;

    // teensy time in ros epoch
    if (init_flag_) {
      strobe.timestamp_ros = 0.0;
    } else {
      strobe.timestamp_ros = strobe.timestamp_teensy + GetTimeOffset();
    }

    // count
    memcpy(&strobe.count, &buf[ind], sizeof(strobe.count));
    // printf("(svis) strobe.count: [%i, %i]\n", ind, strobe.count);
    ind += strobe.count;

    // save packet
    strobe_packets->push_back(strobe);
  }

  timing_.parse_strobe = toc();
}

void SVIS::PushImu(const std::vector<ImuPacket>& imu_packets,
                   boost::circular_buffer<ImuPacket>* imu_buffer) {
  tic();

  ImuPacket imu;
  for (uint i = 0; i < imu_packets.size(); i++) {
    imu = imu_packets[i];
    imu_buffer->push_back(imu);
  }

  // warn if buffer is at max size
  if (imu_buffer->size() == imu_buffer->max_size()) {
    printf("(svis) imu buffer at max size\n");
  }

  timing_.push_imu = toc();
}

void SVIS::PushStrobe(const std::vector<StrobePacket>& strobe_packets,
                      boost::circular_buffer<StrobePacket>* strobe_buffer) {
  tic();

  StrobePacket strobe;
  for (uint i = 0; i < strobe_packets.size(); i++) {
    strobe = strobe_packets[i];
    strobe_buffer->push_back(strobe);
  }

  // warn if buffer is at max size
  if (strobe_buffer->size() == strobe_buffer->max_size()) {
    printf("(svis) strobe buffer at max size\n");
  }

  timing_.push_strobe = toc();
}

void SVIS::DecimateImu(boost::circular_buffer<ImuPacket>* imu_buffer,
                       std::vector<ImuPacket>* imu_packets_filt) {
  // create filter packets
  while (imu_buffer->size() >= static_cast<std::size_t>(imu_filter_size_) && imu_filter_size_ > 0) {
    for (int i = 0; i < imu_filter_size_; i++) {
      if (i == 0) {
        imu_packets_filt->push_back(imu_buffer->front());
      }
      imu_buffer->pop_front();
    }
  }
}

void SVIS::FilterImu(boost::circular_buffer<ImuPacket>* imu_buffer,
                     std::vector<ImuPacket>* imu_packets_filt) {
  tic();

  // create filter packets
  while (imu_buffer->size() >= static_cast<std::size_t>(imu_filter_size_) && imu_filter_size_ > 0) {
    double timestamp_diff_mean = 0.0;
    double acc_mean[3] = {0.0};
    double gyro_mean[3] = {0.0};

    // get local time offset for better precision
    double first_timestamp = imu_buffer->front().timestamp_ros;
    
    for (int i = 0; i < imu_filter_size_; i++) {
      ImuPacket temp_packet = imu_buffer->front();
      imu_buffer->pop_front();

      timestamp_diff_mean += (temp_packet.timestamp_ros - first_timestamp) / static_cast<double>(imu_filter_size_);
      for (uint j = 0; j < 3; j++) {
        acc_mean[j] += temp_packet.acc[j] / static_cast<double>(imu_filter_size_);
        gyro_mean[j] += temp_packet.gyro[j] / static_cast<double>(imu_filter_size_);
      }
    }

    ImuPacket filter_packet;
    filter_packet.timestamp_ros = first_timestamp + timestamp_diff_mean;
    for (uint j = 0; j < 3; j++) {
      filter_packet.acc[j] = acc_mean[j];
      filter_packet.gyro[j] = gyro_mean[j];
    }

    // save packet
    imu_packets_filt->push_back(filter_packet);
  }

  timing_.filter_imu = toc();
}

void SVIS::PrintBuffer(const std::vector<char>& buf) {
  printf("(svis) buffer: ");
  for (uint i = 0; i < buf.size(); i++) {
    printf("%02X ", buf[i] & 255);
  }
  printf("\n");
}

void SVIS::ComputeStrobeTotal(std::vector<StrobePacket>* strobe_packets) {
  tic();

  for (auto& str : (*strobe_packets)) {
    // printf("strobe_count_total: %u\n", strobe_count_total_);

    // initialize variables on first iteration
    if (strobe_count_total_ == 0 && strobe_count_last_ == 0) {
      // printf("init strobe count\n");
      strobe_count_total_ = 1;
      strobe_count_last_ = str.count_total;
      str.count_total = strobe_count_total_;
      continue;
    }

    // get count difference between two most recent strobe messages
    uint8_t diff = 0;
    if (str.count > strobe_count_last_) {
      // no rollover
      diff = str.count - strobe_count_last_;
    } else if (str.count < strobe_count_last_) {
      // rollover
      diff = (strobe_count_last_ + str.count);

      // handle rollover
      if (diff == 255) {
        // printf("(svis) Handle rollover\n");
        diff = 1;
      }
    } else {
      // no change
      printf("(svis) no change in strobe count\n");
    }

    // check diff value
    if (diff > 1 && !std::isinf(strobe_count_last_) && !init_flag_) {
      printf("(svis) detected jump in strobe count\n");
      // printf("(svis) diff: %i, last: %i, count: %i\n",
      //              diff,
      //              strobe_count_last_,
      //              strobe_packets[i].count);
    } else if (diff < 1 && !std::isinf(strobe_count_last_)) {
      printf("(svis) detected lag in strobe count\n");
    }

    // update count
    strobe_count_total_ += diff;

    // set packet total
    str.count_total = strobe_count_total_;

    // update last value
    strobe_count_last_ = str.count;
  }

  timing_.compute_strobe_total = toc();
}

void SVIS::Associate(boost::circular_buffer<StrobePacket>* strobe_buffer,
                     boost::circular_buffer<CameraPacket>* camera_buffer,
                     std::vector<CameraStrobePacket>* camera_strobe_packets) {
  tic();

  // create camera strobe packets
  CameraStrobePacket camera_strobe;
  int fail_count = 0;
  int match_count = 0;
  bool match = false;

  // printf("strobe_buffer size: %lu\n", strobe_buffer->size());
  // printf("camera_buffer size: %lu\n", camera_buffer->size());

  for (auto it_strobe = strobe_buffer->begin(); it_strobe != strobe_buffer->end(); ) {
    // printf("i: %lu\n", std::distance(strobe_buffer->begin(), it_strobe));
    match = false;
    for (auto it_camera = camera_buffer->begin(); it_camera != camera_buffer->end(); ) {
      // printf("j: %lu\n", std::distance(camera_buffer->begin(), it_camera));
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
        camera_strobe_packets->push_back(camera_strobe);

        // remove matched strobe
        // printf("delete matched camera\n");
        it_camera = camera_buffer->erase(it_camera);

        // record match
        match_count++;
        match = true;
        // printf("match\n");
        break;
      } else {
        // check for stale entry and delete
        if ((TimeNow() - (*it_camera).image.header.stamp) > 1.0) {
          // printf("delete stale camera\n");
          it_camera = camera_buffer->erase(it_camera);
        } else {
          // printf("increment camera\n");
          ++it_camera;
        }
      }
    }

    // increment fail count for no images for a given strobe message
    if (match) {
      // remove matched strobe
      // printf("delete matched strobe\n");
      it_strobe = strobe_buffer->erase(it_strobe);
    } else {
      fail_count++;
      // printf("fail\n");

      // check for stale entry and delete
      if ((TimeNow() - (*it_strobe).timestamp_ros_rx) > 1.0) {
        printf("(svis ros) Delete stale strobe\n");
        it_strobe = strobe_buffer->erase(it_strobe);
      } else {
        // printf("increment strobe\n");
        ++it_strobe;
      }
    }
  }

  // printf("final fail_count: %i\n", fail_count);
  // printf("final match_count: %i\n", match_count);
  if (static_cast<std::size_t>(fail_count) == strobe_buffer->max_size()) {
    printf("Failure to match.  Resyncing...\n");
    sync_flag_ = true;
  }

  timing_.associate = toc();
}

void SVIS::PrintCameraBuffer(const boost::circular_buffer<CameraPacket>& camera_buffer) {
  double t_now = TimeNow();
  printf("camera_buffer: %lu\n", camera_buffer.size());
  for (uint i = 0; i < camera_buffer.size(); i++) {
    // printf("%i:(%i)%f ", i, camera_buffer[i].metadata.frame_counter, t_now - camera_buffer[i].image.header.stamp);  // NO INDEXING
  }
  printf("\n\n");
}

void SVIS::PrintStrobeBuffer(const boost::circular_buffer<StrobePacket>& strobe_buffer) {
  double t_now = TimeNow();
  printf("strobe_buffer: %lu\n", strobe_buffer.size());
  for (uint i = 0; i < strobe_buffer.size(); i++) {
    // printf("%i:(%i, %i)%f ", i, strobe_buffer[i].count, strobe_buffer[i].count_total + strobe_count_offset_, t_now - strobe_buffer[i].timestamp_ros);  // NO INDEXING
  }
  printf("\n");
}

// void SVIS::PrintImageQuadlet(const std::string& name,
//                              const sensor_msgs::Image::ConstPtr& msg,
//                              const int& i) {
//   printf("%s: ", name.c_str());
//   printf("%02X ", msg->data[i]);
//   printf("%02X ", msg->data[i + 1]);
//   printf("%02X ", msg->data[i + 2]);
//   printf("%02X ", msg->data[i + 3]);
//   printf("\n");
// }

// void SVIS::PrintMetaDataRaw(const sensor_msgs::Image::ConstPtr& msg) {
//   ROS_INFO("encoding: %s", msg->encoding.c_str());
//   ROS_INFO("step: %i", msg->step);
//   ROS_INFO("width: %i", msg->width);
//   ROS_INFO("height: %i", msg->height);
//   ROS_INFO("is_bigendian: %i", msg->is_bigendian);
//   PrintImageQuadlet("timestamp", msg, 0);
//   PrintImageQuadlet("gain", msg, 4);
//   PrintImageQuadlet("shutter", msg, 8);
//   PrintImageQuadlet("brightness", msg, 12);
//   PrintImageQuadlet("exposure", msg, 16);
//   PrintImageQuadlet("white balance", msg, 20);
//   PrintImageQuadlet("frame counter", msg, 24);
//   PrintImageQuadlet("roi", msg, 28);
//   printf("\n\n");
// }

void SVIS::ParseImageMetadata(const Image& image,
                            CameraPacket* camera_packet) {
  // timestamp
  std::memcpy(&camera_packet->metadata.timestamp, &image.data[0],
              sizeof(camera_packet->metadata.timestamp));

  // gain
  std::memcpy(&camera_packet->metadata.gain, &image.data[4],
              sizeof(camera_packet->metadata.gain));

  // shutter
  std::memcpy(&camera_packet->metadata.shutter, &image.data[8],
              sizeof(camera_packet->metadata.shutter));

  // brightness
  std::memcpy(&camera_packet->metadata.brightness, &image.data[12],
              sizeof(camera_packet->metadata.brightness));

  // exposure
  std::memcpy(&camera_packet->metadata.exposure, &image.data[16],
              sizeof(camera_packet->metadata.exposure));

  // white balance
  std::memcpy(&camera_packet->metadata.white_balance, &image.data[20],
              sizeof(camera_packet->metadata.white_balance));

  // frame counter
  uint32_t frame_counter = 0xFF & image.data[27];
  frame_counter |= (0xFF & image.data[26]) << 8;
  frame_counter |= (0xFF & image.data[25]) << 16;
  frame_counter |= (0xFF & image.data[24]) << 24;
  camera_packet->metadata.frame_counter = frame_counter;

  // region of interest
  std::memcpy(&camera_packet->metadata.roi_position, &image.data[28],
              sizeof(camera_packet->metadata.roi_position));
}

void SVIS::tic() {
  tic_ = std::chrono::high_resolution_clock::now();
}

double SVIS::toc() {
  std::chrono::duration<double> duration = std::chrono::high_resolution_clock::now() - tic_;

  return duration.count();
}

void SVIS::SetPublishStrobeRawHandler(std::function<void(const std::vector<StrobePacket>&)> handler) {
  PublishStrobeRaw = handler;
}

void SVIS::SetPublishImuRawHandler(std::function<void(const std::vector<ImuPacket>&)> handler) {
  PublishImuRaw = handler;
}

void SVIS::SetPublishImuHandler(std::function<void(const ImuPacket&)> handler) {
  PublishImu = handler;
}

void SVIS::SetPublishCameraHandler(std::function<void(std::vector<CameraStrobePacket>&)> handler) {
  PublishCamera = handler;
}

void SVIS::SetPublishTimingHandler(std::function<void(const Timing&)> handler) {
  PublishTiming = handler;
}

void SVIS::SetTimeNowHandler(std::function<double()> handler) {
  TimeNow = handler;
}

}  // namespace svis
