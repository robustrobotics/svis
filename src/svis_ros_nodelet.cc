// Copyright 2016 Massachusetts Institute of Technology

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <termios.h>

#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <pluginlib/class_list_macros.h>

extern "C" {
#include "hid/hid.h"
}

namespace svis_ros {

/**
 * \brief A simple visual inertial synchronization approach.
 *
 * This package contains the ROS portion of a Simple Visual Inertial Synchronization approach that accepts camera strobe messages * from the Teensy microcontroller and synchronizes them with camera image messages.
 */
class SVISNodelet : public nodelet::Nodelet {
 public:
  /**
   * \brief Constructor.
   *
   * NOTE: Default, no-args constructor must exist.
   */
  SVISNodelet() = default;

  ~SVISNodelet() = default;

  SVISNodelet(const SVISNodelet& rhs) = delete;
  SVISNodelet& operator=(const SVISNodelet& rhs) = delete;

  SVISNodelet(SVISNodelet&& rhs) = delete;
  SVISNodelet& operator=(SVISNodelet&& rhs) = delete;

  /**
   * \brief Nodelet initialization.
   *
   * Subclasses of nodelet::Nodelet need to override this virtual method.
   * It takes the place of the nodelet constructor.
   */
  virtual void onInit() {
    // Grab a handle to the parent node.
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // image transport
    image_transport::ImageTransport it(nh);

    // subscribers
    image_sub_ = it.subscribeCamera("/flea3/image_raw", 10, &SVISNodelet::ImageCallback, this);

    // publishers
    image_pub_ = it.advertiseCamera("/flea3/image_raw_sync", 1);
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("/svis/imu", 1);

    // initialize variables
    imu_buffer_.set_capacity(10);
    imu_filter_size_ = 5;
    strobe_buffer_.set_capacity(10);
    camera_buffer_.set_capacity(10);
    init_flag_ = true;
    time_offset_ = 0;

    Run();

    return;
  }

  /**
   * \brief Main processing loop.
   */
  void Run() {
    // open rawhid port
    int r = rawhid_open(1, 0x16C0, 0x0486, 0xFFAB, 0x0200);
    // C-based example is 16C0:0480:FFAB:0200
    // Arduino-based example is 16C0:0486:FFAB:0200

    // check return
    if (r <= 0) {
      NODELET_ERROR("(svis_ros) No svis_teensy device found.\n");
      return;
    } else {
      NODELET_INFO("(svis_ros) Found svis_teensy device\n");
    }

    // loop
    std::vector<char> buf(64);
    while (ros::ok()) {
      // check if any Raw HID packet has arrived
      int num = rawhid_recv(0, buf.data(), buf.size(), 220);

      // timing
      t_now_ = ros::Time::now();
      // NODELET_INFO("(svis_ros) [n: %i, dt: %f]", num, (t_now - t_last).toSec());
      t_last_ = t_now_;

      // check byte count
      if (num < 0) {
        NODELET_ERROR("(svis_ros): Error reading, device went offline");
        rawhid_close(0);
        return;
      } else if (num == 0) {
        NODELET_INFO("(svis_ros) 0");
      } else if (num > 0) {
        // resize vector
        buf.resize(num);

        // debug print
        if (print_buffer_) {
          PrintBuffer(buf);
        }

        // checksum
        if (GetChecksum(buf)) {
          return;
        }

        // header
        HeaderPacket header;
        GetHeader(buf, header);

        // get data
        GetIMU(buf, header);
        GetStrobe(buf, header);

        if (init_flag_) {
          GetOffset();
          continue;
        }

        // filter imu
        FilterIMU();

        // associate strobe and camera
        AssociateStrobe();

        // publish
        PublishIMU();
        PublishCamera();
      } else {
        NODELET_WARN("(svis_ros) Bad return value from rawhid_recv");
      }
    }
  }

 private:
  class HeaderPacket {
   public:
    double timestamp_ros_rx;  // time message was received
    uint16_t send_count;
    uint8_t imu_count;
    uint8_t strobe_count;
  };

  class StrobePacket {
   public:
    double timestamp_ros_rx;  // [seconds] time usb message was received in ros epoch
    double timestamp_ros;  // [seconds] timestamp in ros epoch
    uint32_t timestamp_teensy_raw;  // [microseconds] timestamp in teensy epoch
    double timestamp_teensy;  // [seconds] timestamp in teensy epoch
    uint8_t count;  // number of camera images
  };

  class ImuPacket {
   public:
    double timestamp_ros_rx;  // [seconds] time usb message was received in ros epoch
    double timestamp_ros;  // [seconds] timestamp in ros epoch
    uint32_t timestamp_teensy_raw;  // [microseconds] timestamp in teensy epoch
    double timestamp_teensy;  // [seconds] timestamp in teensy epoch
    int16_t acc[3];  // units?
    int16_t gyro[3];  // units?
  };

  class CameraPacket {
   public:
    double gain;
    double shutter;
    double brightness;
    double exposure;
    double frame_counter;
    double strobe_pattern;
    double gpio_state;
    double roi_position;
    sensor_msgs::CameraInfo::ConstPtr info;
    sensor_msgs::Image::ConstPtr image;
  };

  class CameraStrobePacket {
   public:
    CameraPacket image;
    StrobePacket strobe;
  };

  int GetChecksum(std::vector<char> &buf) {
    // calculate checksum
    uint16_t checksum_calc = 0;
    for (int i = 0; i < send_buffer_size - 2; i++) {
      checksum_calc += buf[i] & 0xFF;
    }

    // get packet chechsum
    uint16_t checksum_orig = 0;
    memcpy(&checksum_orig, &buf[checksum_index], sizeof(checksum_orig));

    // check match and return result
    if (checksum_calc != checksum_orig) {
      NODELET_INFO("(svis_ros) checksum error [%02X, %02X] [%02X, %02X]",
                   buf[checksum_index] & 0xFF, buf[checksum_index + 1] & 0xFF, checksum_calc, checksum_orig);
      return 1;
    } else {
      return 0;
    }
  }

  void GetOffset() {
    if (init_flag_) {
      if (time_offset_vec_.size() >= 1000) {
        // filter initial values that are often composed of stale data
        NODELET_INFO("time_offset_vec.size(): %lu", time_offset_vec_.size());
        while (fabs(time_offset_vec_.front() - time_offset_vec_.back()) > 0.1) {
          time_offset_vec_.pop_front();
        }
        NODELET_INFO("filtered time_offset_vec.size(): %lu", time_offset_vec_.size());
        
        // sum time offsets
        double sum = 0.0;
        for (int i = 0; i < time_offset_vec_.size(); i++) {
          sum += time_offset_vec_[i];
        }

        // calculate final time offset
        time_offset_ = sum / static_cast<double>(time_offset_vec_.size());
        NODELET_INFO("time_offset: %f", time_offset_);
        init_flag_ = false;
      } else {
        // use third imu packet since it triggers send
        ImuPacket imu = imu_buffer_[2];
        time_offset_vec_.push_back(imu.timestamp_ros_rx - imu.timestamp_teensy);
        // NODELET_INFO("now: %f, rx: %f, teensy: %f, offset: %f, ros: %f",
        //              ros::Time::now().toSec(),
        //              imu.timestamp_ros_rx,
        //              imu.timestamp_teensy,
        //              imu.timestamp_ros_rx - imu.timestamp_teensy,
        //              imu.timestamp_ros);
        imu_buffer_.clear();
      }
    }
  }

  void GetHeader(std::vector<char> &buf, HeaderPacket &header) {
    int ind = 0;

    // ros time
    header.timestamp_ros_rx = ros::Time::now().toSec();

    // send_count
    memcpy(&header.send_count, &buf[ind], sizeof(header.send_count));
    // NODELET_INFO("(svis_ros) send_count: [%i, %i]", ind, header.send_count);
    ind += sizeof(header.send_count);

    // imu_packet_count
    memcpy(&header.imu_count, &buf[ind], sizeof(header.imu_count));
    // NODELET_INFO("(svis_ros) imu_packet_count: [%i, %i]", ind, header.imu_count);
    ind += sizeof(header.imu_count);

    // strobe_packet_count
    memcpy(&header.strobe_count, &buf[ind], sizeof(header.strobe_count));
    // NODELET_INFO("(svis_ros) strobe_packet_count: [%i, %i]", ind, header.strobe_count);
    ind += sizeof(header.strobe_count);
  }

  void GetIMU(std::vector<char> &buf, HeaderPacket &header) {
    for (int i = 0; i < header.imu_count; i++) {
      ImuPacket imu;
      int ind = imu_index[i];

      // ros receive time
      imu.timestamp_ros_rx = header.timestamp_ros_rx;

      // raw teensy timestamp
      memcpy(&imu.timestamp_teensy_raw, &buf[ind], sizeof(imu.timestamp_teensy_raw));
      // NODELET_INFO("(svis_ros) imu.timestamp: [%i, %i]", ind, imu.timestamp);
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
      memcpy(&imu.acc[0], &buf[ind], sizeof(imu.acc[0]));
      ind += sizeof(imu.acc[0]);
      memcpy(&imu.acc[1], &buf[ind], sizeof(imu.acc[1]));
      ind += sizeof(imu.acc[1]);
      memcpy(&imu.acc[2], &buf[ind], sizeof(imu.acc[2]));
      ind += sizeof(imu.acc[2]);

      // NODELET_INFO("(svis_ros) imu.acc: [%i, %i, %i]", imu.acc[0], imu.acc[1], imu.acc[2]);

      // gyro
      memcpy(&imu.gyro[0], &buf[ind], sizeof(imu.gyro[0]));
      ind += sizeof(imu.gyro[0]);
      memcpy(&imu.gyro[1], &buf[ind], sizeof(imu.gyro[1]));
      ind += sizeof(imu.gyro[1]);
      memcpy(&imu.gyro[2], &buf[ind], sizeof(imu.gyro[2]));
      ind += sizeof(imu.gyro[2]);

      // NODELET_INFO("(svis_ros) imu.gyro: [%i, %i, %i]", imu.gyro[0], imu.gyro[1], imu.gyro[2]);

      // save packet
      imu_buffer_.push_back(imu);
    }
  }

  void GetStrobe(std::vector<char> &buf, HeaderPacket &header) {
    for (int i = 0; i < header.strobe_count; i++) {
      StrobePacket strobe;
      int ind = strobe_index[i];

      // ros time
      strobe.timestamp_ros_rx = header.timestamp_ros_rx;

      // timestamp
      memcpy(&strobe.timestamp_teensy_raw, &buf[ind], sizeof(strobe.timestamp_teensy_raw));
      // NODELET_INFO("(svis_ros) strobe.timestamp: [%i, %i]", ind, strobe.timestamp);
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
      // NODELET_INFO("(svis_ros) strobe.count: [%i, %i]", ind, strobe.count);
      ind += strobe.count;

      // save packet
      strobe_buffer_.push_back(strobe);

      // increment global strobe count
      strobe_count_++;
    }
  }

  void FilterIMU() {
    // create filter packets
    while (imu_buffer_.size() >= imu_filter_size_) {
      // sum
      double timestamp_total = 0.0;
      double acc_total[3] = {0.0};
      double gyro_total[3] = {0.0};
      ImuPacket temp_packet;
      for (int i = 0; i < imu_filter_size_; i++) {
        temp_packet = imu_buffer_[0];
        imu_buffer_.pop_front();

        timestamp_total += static_cast<double>(temp_packet.timestamp_teensy);
        for (int j = 0; j < 3; j++) {
          acc_total[j] += static_cast<double>(temp_packet.acc[j]);
          gyro_total[j] += static_cast<double>(temp_packet.gyro[j]);
        }
      }

      // calculate average
      temp_packet.timestamp_teensy =
        static_cast<int>(timestamp_total / static_cast<double>(imu_filter_size_));
      for (int j = 0; j < 3; j++) {
        temp_packet.acc[j] = static_cast<int>(acc_total[j] / static_cast<double>(imu_filter_size_));
        temp_packet.gyro[j] =
          static_cast<int>(gyro_total[j] / static_cast<double>(imu_filter_size_));
      }
      imu_packets_filt_.push_back(temp_packet);
    }
  }

  void PublishIMU() {
    ImuPacket temp_packet;
    sensor_msgs::Imu imu;

    for (int i = 0; i < imu_packets_filt_.size(); i++) {
      temp_packet = imu_packets_filt_[i];

      imu.header.stamp = ros::Time(temp_packet.timestamp_teensy + time_offset_);
      imu.header.frame_id = "body";

      // orientation
      imu.orientation.x = std::numeric_limits<double>::quiet_NaN();
      imu.orientation.y = std::numeric_limits<double>::quiet_NaN();
      imu.orientation.z = std::numeric_limits<double>::quiet_NaN();
      imu.orientation.w = std::numeric_limits<double>::quiet_NaN();

      // orientation covariance
      for (int i = 0; i < 9; i++) {
        imu.orientation_covariance[i] = std::numeric_limits<double>::quiet_NaN();
      }

      // angular velocity
      imu.angular_velocity.x = temp_packet.gyro[0];
      imu.angular_velocity.y = temp_packet.gyro[1];
      imu.angular_velocity.z = temp_packet.gyro[2];

      // angular velocity covariance
      for (int i = 0; i < 9; i++) {
        imu.angular_velocity_covariance[i] = std::numeric_limits<double>::quiet_NaN();
      }

      // linear acceleration
      imu.linear_acceleration.x = temp_packet.acc[0];
      imu.linear_acceleration.y = temp_packet.acc[1];
      imu.linear_acceleration.z = temp_packet.acc[2];

      // acceleration covariance
      for (int i = 0; i < 9; i++) {
        imu.linear_acceleration_covariance[i] = std::numeric_limits<double>::quiet_NaN();
      }

      // publish
      imu_pub_.publish(imu);
    }

    // clear buffer
    imu_packets_filt_.clear();
  }

  void PrintBuffer(std::vector<char> &buf) {
    NODELET_INFO("(svis_ros) buffer: ");
    for (int i = 0; i < buf.size(); i++) {
      printf("%02X ", buf[i] & 255);
    }
    printf("\n");
  }

  void PrintImageQuadlet(std::string name, const sensor_msgs::Image::ConstPtr& msg, int i) {
    printf("%s: ", name.c_str());
    printf("%02X ", msg->data[i]);
    printf("%02X ", msg->data[i + 1]);
    printf("%02X ", msg->data[i + 2]);
    printf("%02X ", msg->data[i + 3]);
    printf("\n");
  }

  void PrintMetaDataRaw(const sensor_msgs::Image::ConstPtr& msg) {
    NODELET_INFO("encoding: %s", msg->encoding.c_str());
    NODELET_INFO("step: %i", msg->step);
    NODELET_INFO("width: %i", msg->width);
    NODELET_INFO("height: %i", msg->height);
    NODELET_INFO("is_bigendian: %i", msg->is_bigendian);
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

  void ImageCallback(const sensor_msgs::Image::ConstPtr& image_msg,
                     const sensor_msgs::CameraInfo::ConstPtr& info_msg) {
    PrintMetaDataRaw(image_msg);
    NODELET_INFO("strobe_count: %i", strobe_count_);

    // store image
    CameraPacket camera_packet;
    camera_packet.image = image_msg;
    camera_packet.info = info_msg;
    camera_buffer_.push_back(camera_packet);
  }

  void AssociateStrobe() {
    // not yet implemented
  }

  void PublishCamera() {
    for (int i = 0; i < camera_strobe_packets_.size(); i++) {
      // publish info message
      sensor_msgs::CameraInfo info_msg;

      // publish image message
      sensor_msgs::Image image_msg;
    }

    camera_strobe_packets_.clear();
  }

  // publishers
  image_transport::CameraPublisher image_pub_;
  ros::Publisher imu_pub_;

  // subscribers
  image_transport::CameraSubscriber image_sub_;

  // imu
  boost::circular_buffer<ImuPacket> imu_buffer_;
  int imu_filter_size_;
  std::vector<ImuPacket> imu_packets_filt_;

  // camera
  boost::circular_buffer<StrobePacket> strobe_buffer_;
  boost::circular_buffer<CameraPacket> camera_buffer_;
  std::vector<CameraStrobePacket> camera_strobe_packets_;
  std::deque<double> time_offset_vec_;
  double time_offset_;
  int init_count_;
  bool init_flag_;
  ros::Time init_start_;
  ros::Time init_stop_;
  int strobe_count_;

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
  ros::Time t_now_;
  ros::Time t_last_;
};

}  // namespace svis_ros

// Export as a plugin.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(svis_ros::SVISNodelet, nodelet::Nodelet)
