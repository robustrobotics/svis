

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
#include "hid.h"
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
    imu_buffer_.set_capacity(imu_buffer_size);
    imu_filter_size_ = 5;

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
    ros::Time t_now;
    ros::Time t_last;
    while (ros::ok()) {
      // check if any Raw HID packet has arrived
      int num = rawhid_recv(0, buf.data(), buf.size(), 220);

      // timing
      t_now = ros::Time::now();
      // NODELET_INFO("(svis_ros) [n: %i, dt: %f]", num, (t_now - t_last).toSec());
      t_last = t_now;

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
        if(GetChecksum(buf)) {
          return;
        }
        
        // header
        header_packet header;
        GetHeader(buf, header);

        // imu
        std::vector<imu_packet> imu_packets(header.imu_count);
        GetIMU(buf, header, imu_packets);
        PushIMU(imu_packets);

        std::vector<imu_packet> imu_packets_filt;
        FilterIMU(imu_packets_filt, imu_packets_filt);
        PublishIMU(imu_packets_filt);

        // strobe
        std::vector<strobe_packet> strobe_packets(header.strobe_count);
        GetStrobe(buf, header, strobe_packets);

        // new line
        // printf("\n");
      } else {
        NODELET_WARN("(svis_ros) Bad return value from rawhid_recv");
      }
      
    }
  }

 private:
  struct header_packet {
    uint16_t send_count;
    uint8_t imu_count;
    uint8_t strobe_count;
  };

  struct strobe_packet {
    uint32_t timestamp;  // microseconds since teensy bootup
    uint8_t count;  // number of camera images
  };

  struct imu_packet {
    uint32_t timestamp;  // microseconds since teensy bootup
    int16_t acc[3];  // units?
    int16_t gyro[3];  // units?
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

  void GetHeader(std::vector<char> &buf, header_packet &header) {
    int ind = 0;
    
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

  void GetIMU(std::vector<char> &buf, header_packet &header, std::vector<imu_packet> &imu_packets) {
    t_now = ros::Time::now();
    double dt = (t_now - t_last).toSec();
    t_last = t_now;
    ROS_INFO("dt: %0.6f", dt);

    for (int i = 0; i < header.imu_count; i++) {
      imu_packet imu;
      int ind = imu_index[i];

      // timestamp
      memcpy(&imu.timestamp, &buf[ind], sizeof(imu.timestamp));
      // NODELET_INFO("(svis_ros) imu.timestamp: [%i, %i]", ind, imu.timestamp);
      ind += sizeof(imu.timestamp);

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
      imu_packets[i] = imu;
    }
  }

  void GetStrobe(std::vector<char> &buf, header_packet &header, std::vector<strobe_packet> &strobe_packets) {
    for (int i = 0; i < header.strobe_count; i++) {
      strobe_packet strobe;
      int ind = strobe_index[i];
      
      // timestamp
      memcpy(&strobe.timestamp, &buf[ind], sizeof(strobe.timestamp));
      // NODELET_INFO("(svis_ros) strobe.timestamp: [%i, %i]", ind, strobe.timestamp);
      ind += sizeof(strobe.timestamp);

      // count
      memcpy(&strobe.count, &buf[ind], sizeof(strobe.count));
      // NODELET_INFO("(svis_ros) strobe.count: [%i, %i]", ind, strobe.count);
      ind += strobe.count;

      // save packet
      strobe_packets[i] = strobe;
    }
  }

  void PushIMU(std::vector<imu_packet> &imu_packets) {
    // insert imu packets into circular buffer
    for (int i = 0; i < imu_packets.size(); i++) {
      imu_buffer_.push_back(imu_packets[i]);
    }
  }

  void FilterIMU(std::vector<imu_packet> &imu_packets, std::vector<imu_packet> &imu_packets_filt) {
    // create filter packets
    while (imu_buffer_.size() >= imu_filter_size_) {
      
      // sum
      double timestamp_total = 0.0;
      double acc_total[3] = {0.0};
      double gyro_total[3] = {0.0};
      imu_packet temp_packet;
      for (int i = 0; i < imu_filter_size_; i++) {
        temp_packet = imu_buffer_[0];
        imu_buffer_.pop_front();
        
        timestamp_total += static_cast<double>(temp_packet.timestamp);
        for (int j = 0; j < 3; j++) {
          acc_total[j] += static_cast<double>(temp_packet.acc[j]);
          gyro_total[j] += static_cast<double>(temp_packet.gyro[j]);
        }
      }

      // calculate average
      temp_packet.timestamp = static_cast<int>(timestamp_total / static_cast<double>(imu_filter_size_));
      for (int j = 0; j < 3; j++) {
        temp_packet.acc[j] = static_cast<int>(acc_total[j] / static_cast<double>(imu_filter_size_));
        temp_packet.gyro[j] = static_cast<int>(gyro_total[j] / static_cast<double>(imu_filter_size_));
      }
      imu_packets_filt.push_back(temp_packet);
    }
  }

  void PublishIMU(std::vector<imu_packet> &imu_packets_filt) {
    imu_packet temp_packet;
    sensor_msgs::Imu imu;
    
    for (int i = 0; i < imu_packets_filt.size(); i++) {
      temp_packet = imu_packets_filt[i];

      imu.header.stamp = ros::Time(temp_packet.timestamp);
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

  void ImageCallback(const sensor_msgs::Image::ConstPtr& msg, const sensor_msgs::CameraInfo::ConstPtr& info) {
    // PrintMetaDataRaw(msg);

    uint32_t timestamp;
    // memcpy(&checksum_orig, &buf[checksum_index], sizeof(checksum_orig));
    // sensor_msgs::Image img = *msg;

    // for (int i = 0; i < 40; i++) {
    //   img.data[i] = 0;
    // }

    // image_pub_.publish(img);
  }

  // publishers
  image_transport::CameraPublisher image_pub_;
  ros::Publisher imu_pub_;

  // subscribers
  image_transport::CameraSubscriber image_sub_;

  // imu buffer
  boost::circular_buffer<imu_packet> imu_buffer_;
  int imu_filter_size_;
  
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
  ros::Time t_now;
  ros::Time t_last;
};

}  // namespace svis_ros

// Export as a plugin.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(svis_ros::SVISNodelet, nodelet::Nodelet)
