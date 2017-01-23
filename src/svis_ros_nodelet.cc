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

#define IMU_DATA_SIZE 6  // (int16_t) [ax, ay, az, gx, gy, gz]
#define IMU_BUFFER_SIZE 10  // store 10 samples (imu_stamp, imu_data) in circular buffers
#define IMU_PACKET_SIZE 17  // (int8_t) [2, imu_stamp[0], ... , imu_stamp[3], imu_data[0], ... , imu_data[11]]
#define STROBE_BUFFER_SIZE 10  // store 10 samples (strobe_stamp, strobe_count) in circular buffers
#define STROBE_PACKET_SIZE 6  // (int8_t) [1, strobe_stamp[0], ... , strobe_stamp[3], strobe_count]
#define SEND_BUFFER_SIZE 64  // (int8_t) size of HID USB packets
#define SEND_HEADER_SIZE 8  // (int8_t) [header1, header2, packet_count[0], ... , packet_count[3], imu_count, strobe_count];

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
        for (i = 0; i < num; i++) {
          // printf("%02X ", buf[i] & 255);
        }

        int ind = 0;
        int mask8  = 0x00000000000000FF;
        int mask16 = 0x000000000000FFFF;
        int mask32 = 0x00000000FFFFFFFF;

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
