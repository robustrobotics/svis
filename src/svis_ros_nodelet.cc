// Copyright 2016 Massachusetts Institute of Technology

#include <memory>
#include <limits>
#include <thread>
#include <vector>

#include <Eigen/Core>

#include <sophus/se3.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <ros/ros.h>

#include <nodelet/nodelet.h>

#include <ros_sensor_streams/tracked_image_stream.h>

#include <stats_tracker/stats_tracker.h>

#include <depthest/trackers/track_manager.h>
#include <depthest/trackers/klt_feature_tracker.h>

namespace dft = depthest::trackers;

namespace depthest_ros {

/**
 * \brief Tracks sparse features using OpenCV KLT tracker.
 *
 * KLT tracker is essentially Lucas-Kanade optical flow, but with features that
 * are "good" for tracking.
 *
 * Derived from:
 * http://docs.opencv.org/3.1.0/d7/d8b/tutorial_py_lucas_kanade.html#gsc.tab=0
 */
class FeatureTrackerNodelet : public nodelet::Nodelet {
 public:
  /**
   * \brief Constructor.
   *
   * NOTE: Default, no-args constructor must exist.
   */
  FeatureTrackerNodelet() = default;

  virtual ~FeatureTrackerNodelet() {
    if (thread_.joinable()) {
      thread_.join();
    }

    return;
  }

  FeatureTrackerNodelet(const FeatureTrackerNodelet& rhs) = delete;
  FeatureTrackerNodelet& operator=(const FeatureTrackerNodelet& rhs) = delete;

  FeatureTrackerNodelet(FeatureTrackerNodelet&& rhs) = delete;
  FeatureTrackerNodelet& operator=(FeatureTrackerNodelet&& rhs) = delete;

  /**
   * \brief Nodelet initialization.
   *
   * Subclasses of nodelet::Nodelet need to override this virtual method.
   * It takes the place of the nodelet constructor.
   */
  virtual void onInit() {
    // Grab a handle to the parent node.
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& pnh = getPrivateNodeHandle();

    // Grab some params.
    if (!pnh.getParam("camera_world_frame_id", camera_world_frame_id_)) {
      NODELET_ERROR("camera_world_frame_id must be specified!");
      return;
    }

    float max_dropouts;
    if (!pnh.getParam("max_dropouts", max_dropouts)) {
      NODELET_ERROR("max_dropouts must be specified!");
      return;
    }

    // Initialize tracker.
    manager_ = std::make_shared<dft::TrackManager>();

    dft::KLTFeatureTracker::Params params;
    params.max_dropouts = max_dropouts;
    tracker_ = std::make_shared<dft::KLTFeatureTracker>(manager_.get(), params);

    // Setup input stream.
    input_ = std::make_shared<ros_sensor_streams::
                              TrackedImageStream>(camera_world_frame_id_, nh);

    // Wait until input is initialized.
    while (!input_->inited()) {
      std::this_thread::yield();
    }

    Kinv_ = input_->K().inverse();

    num_imgs_ = 0;

    thread_ = std::thread(&FeatureTrackerNodelet::run, this);

    return;
  }

  /**
   * \brief Main processing loop.
   */
  void run() {
    while (ros::ok()) {
      stats_.tick("run");

      // Wait for queue to have items.
      std::unique_lock<std::recursive_mutex> lock(input_->queue().mutex());
      input_->queue().non_empty().wait(lock, [this](){
          return (input_->queue().size() > 0);
        });
      lock.unlock();

      // Grab the first item in the queue.
      auto item = input_->queue().pop();

      double time = std::get<0>(item);
      cv::Mat rgb = std::get<1>(item);
      Eigen::Quaternionf q = std::get<2>(item);
      Eigen::Vector3f t = std::get<3>(item);

      Sophus::SE3f pose(q, t);

      // Eat data.
      process(time, pose, rgb);

      num_imgs_++;

      stats_.tock("run");

      NODELET_DEBUG("run = %3.1fms\n", stats_.timings("run"));
    }

    return;
  }

  void process(const double time, const Sophus::SE3f& pose,
               const cv::Mat& img) {
    stats_.tick("process");

    // Convert image to grayscale, if necessary
    cv::Mat img_gray;
    if (img.channels() == 1) {
      img_gray = img;
    } else {
      cv::cvtColor(img, img_gray, cv::COLOR_RGB2GRAY);
    }

    // Assert that we work with 8 bit images
    assert(img_gray.elemSize() == 1);

    // Track.
    tracker_->track(num_imgs_, img_gray);

    NODELET_DEBUG("num_tracks = %i\n",
                  static_cast<int>(manager_->tracks().size()));

    stats_.tock("process");

    NODELET_DEBUG("process = %3.1fms\n", stats_.timings("process"));

    return;
  }

 private:
  // Main processing thread.
  std::thread thread_;

  // Keeps track of statistics and timings.
  stats_tracker::StatsTracker stats_;

  // Number of images processed.
  int num_imgs_;

  // Frame id of the world in camera (Right-Down-Forward) coordinates.
  std::string camera_world_frame_id_;

  // Input stream object.
  std::shared_ptr<ros_sensor_streams::TrackedImageStream> input_;
  Eigen::Matrix3f Kinv_;

  // Feature tracker.
  std::shared_ptr<dft::TrackManager> manager_;
  std::shared_ptr<dft::FeatureTracker> tracker_;
};

}  // namespace depthest_ros

// Export as a plugin.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depthest_ros::FeatureTrackerNodelet, nodelet::Nodelet)
