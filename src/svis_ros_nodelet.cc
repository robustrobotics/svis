// Copyright 2016 Massachusetts Institute of Technology

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <termios.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
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

    Run();

    return;
  }

  /**
   * \brief Main processing loop.
   */
  void Run() {
    int i, r, num;
    char buf[64];

    // C-based example is 16C0:0480:FFAB:0200
    // Arduino-based example is 16C0:0486:FFAB:0200
    r = rawhid_open(1, 0x16C0, 0x0486, 0xFFAB, 0x0200);
    if (r <= 0) {
      NODELET_ERROR("(svis_ros) No rawhid device found.\n");
      return;
    }
    NODELET_INFO("(svis_ros) Found rawhid device\n");

    ros::Time t_now;
    ros::Time t_last;
    while (ros::ok()) {
      // check if any Raw HID packet has arrived
      num = rawhid_recv(0, buf, 64, 220);
      t_now = ros::Time::now();
      // NODELET_INFO("(svis_ros) [n: %i, dt: %f]", num, (t_now - t_last).toSec());
      t_last = t_now;
      if (num < 0) {
        NODELET_ERROR("(svis_ros): Error reading, device went offline");
        rawhid_close(0);
        return;
      } else if (num == 0) {
        NODELET_INFO("(svis_ros) 0");
      } else if (num > 0) {
        // NODELET_INFO("(svis_ros) buffer: ");
        // for (i = 0; i < num; i++) {
        //   printf("%02X ", buf[i] & 255);
        // }
        // printf("\n");

        int ind = 0;
        int mask8  = 0x00000000000000FF;
        int mask16 = 0x000000000000FFFF;
        int mask32 = 0x00000000FFFFFFFF;

        // checksum
        uint16_t checksum_calc = 0;
        for (int i = 0; i < send_buffer_size - 2; i++) {
          checksum_calc += buf[i] & 0xFF;
        }
        uint16_t checksum_orig = 0;
        memcpy(&checksum_orig, &buf[checksum_index], sizeof(checksum_orig));

        if (checksum_calc != checksum_orig) {
          NODELET_INFO("(svis_ros) checksum error [%02X, %02X] [%02X, %02X]",
                       buf[checksum_index] & 0xFF, buf[checksum_index + 1] & 0xFF, checksum_calc, checksum_orig);
          continue;
        }

        // header
        header_packet header;

        // send_count
        header.send_count = mask16 & (mask8 & buf[ind] | ((mask8 & buf[ind + 1]) << 8));
        NODELET_INFO("(svis_ros) send_count: [%i, %i]", ind, header.send_count);
        ind += 2;

        // imu_packet_count
        header.imu_count = mask8 & buf[ind];
        NODELET_INFO("(svis_ros) imu_packet_count: [%i, %i]", ind, header.imu_count);
        ind++;

        // strobe_packet_count
        header.strobe_count = mask8 & buf[ind];
        NODELET_INFO("(svis_ros) strobe_packet_count: [%i, %i]", ind, header.strobe_count);
        ind++;

        // imu
        std::vector<imu_packet> imu_packets(header.imu_count);
        for (int i = 0; i < header.imu_count; i++) {
          imu_packet imu;

          // timestamp
          memcpy(&imu.timestamp, &buf[ind], 4);
          NODELET_INFO("(svis_ros) imu.timestamp: [%i, %i]", ind, imu.timestamp);
          ind += 4;

          // accel
          imu.acc[0] = mask16 & (mask8 & buf[ind] | ((mask8 & buf[ind + 1]) << 8));
          ind += 2;
          imu.acc[1] = mask16 & (mask8 & buf[ind] | ((mask8 & buf[ind + 1]) << 8));
          ind += 2;
          imu.acc[2] = mask16 & (mask8 & buf[ind] | ((mask8 & buf[ind + 1]) << 8));
          ind += 2;

          NODELET_INFO("(svis_ros) imu.acc: [%i, %i, %i]", imu.acc[0], imu.acc[1], imu.acc[2]);

          // gyro
          imu.gyro[0] = mask16 & (mask8 & buf[ind] | ((mask8 & buf[ind + 1]) << 8));
          ind += 2;
          imu.gyro[1] = mask16 & (mask8 & buf[ind] | ((mask8 & buf[ind + 1]) << 8));
          ind += 2;
          imu.gyro[2] = mask16 & (mask8 & buf[ind] | ((mask8 & buf[ind + 1]) << 8));
          ind += 2;

          NODELET_INFO("(svis_ros) imu.gyro: [%i, %i, %i]", imu.gyro[0], imu.gyro[1], imu.gyro[2]);
        }

        // strobe
        std::vector<strobe_packet> strobe_packets(header.strobe_count);
        for (int i = 0; i < header.strobe_count; i++) {
          strobe_packet strobe;

          // timestamp
          memcpy(&strobe.timestamp, &buf[ind], 4);
          NODELET_INFO("(svis_ros) strobe.timestamp: [%i, %i]", ind, strobe.timestamp);
          ind += 4;

          // count
          strobe.count = mask8 & buf[ind];
          NODELET_INFO("(svis_ros) strobe.count: [%i, %i]", ind, strobe.count);
          ind++;
        }

        // new line
        printf("\n");
      } else {
        NODELET_WARN("(svis_ros) Bad return value from rawhid_recv");
      }
      
    }
  }

 private:
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
  
  struct header_packet {
    int send_count;
    int imu_count;
    int strobe_count;
  };

  struct strobe_packet {
    int timestamp;  // microseconds since teensy bootup
    int count;  // number of camera images
  };

  struct imu_packet {
    int timestamp;  // microseconds since teensy bootup
    int acc[3];  // units?
    int gyro[3];  // units?
  };
};

}  // namespace svis_ros

// Export as a plugin.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(svis_ros::SVISNodelet, nodelet::Nodelet)
