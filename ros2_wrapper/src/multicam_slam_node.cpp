// Minimal ROS2 wrapper node around the pre-built dual-camera Multi_ORB_SLAM
// (ORB-SLAM2 based) core library.
//
// Subscribes to two synchronized RGB-D camera pairs (front/rear), feeds
// each synchronized set of frames to ORB_SLAM2::System::TrackRGBD(), and
// republishes the resulting camera pose as nav_msgs::msg::Odometry. The
// original ROS1 ros_rgbd.cc entry point (Examples/ROS/ORB_SLAM2) never
// published pose live -- it only wrote a TUM-format trajectory file on
// shutdown -- so the odometry publisher here is new functionality, not a
// port. See ros2_wrapper/README.md for the exact parameters, topics and
// message shape.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <opencv2/core/core.hpp>
#include <Eigen/Geometry>

#include "System.h"

class MulticamSlamNode : public rclcpp::Node
{
  using ImageMsg = sensor_msgs::msg::Image;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    ImageMsg, ImageMsg, ImageMsg, ImageMsg>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

public:
  MulticamSlamNode()
  : Node("multicam_slam_node")
  {
    // No hardcoded defaults for the SLAM system inputs: these must be
    // supplied at launch, e.g.
    //   ros2 run multicam_slam_ros2_wrapper multicam_slam_node
    //     --ros-args -p vocabulary_file:=/path/to/ORBvoc.txt
    //                -p settings_file:=/path/to/multi.yaml
    //                -p calibration_file:=/path/to/calibration.txt
    this->declare_parameter<std::string>("vocabulary_file", "");
    this->declare_parameter<std::string>("settings_file", "");
    this->declare_parameter<std::string>("calibration_file", "");

    // Topic names default to the same names the original ROS1 ros_rgbd.cc
    // subscribed to, but are overridable via ROS2 parameters.
    this->declare_parameter<std::string>("camera1_rgb_topic", "/camera_01/rgb/image_raw");
    this->declare_parameter<std::string>("camera1_depth_topic", "/camera_01/depth/image_raw");
    this->declare_parameter<std::string>("camera2_rgb_topic", "/camera_02/rgb/image_raw");
    this->declare_parameter<std::string>("camera2_depth_topic", "/camera_02/depth/image_raw");

    const std::string vocab_file = this->get_parameter("vocabulary_file").as_string();
    const std::string settings_file = this->get_parameter("settings_file").as_string();
    const std::string calibration_file = this->get_parameter("calibration_file").as_string();

    if (vocab_file.empty() || settings_file.empty() || calibration_file.empty()) {
      RCLCPP_FATAL(
        this->get_logger(),
        "'vocabulary_file', 'settings_file' and 'calibration_file' "
        "parameters are all required and must be non-empty.");
      throw std::runtime_error("Missing required ORB_SLAM2 parameters.");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Loading Multi_ORB_SLAM (vocab: %s, settings: %s, calibration: %s)",
      vocab_file.c_str(), settings_file.c_str(), calibration_file.c_str());

    // RGBD mode, viewer disabled (this fork's core library is built
    // without Pangolin -- see the top-level CMakeLists.txt Pangolin-optional
    // change).
    slam_system_ = std::make_unique<ORB_SLAM2::System>(
      vocab_file, settings_file, calibration_file, ORB_SLAM2::System::RGBD,
      /*bUseViewer=*/false);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/multicam_slam/odom", 10);

    const std::string cam1_rgb = this->get_parameter("camera1_rgb_topic").as_string();
    const std::string cam1_depth = this->get_parameter("camera1_depth_topic").as_string();
    const std::string cam2_rgb = this->get_parameter("camera2_rgb_topic").as_string();
    const std::string cam2_depth = this->get_parameter("camera2_depth_topic").as_string();

    rgb1_sub_.subscribe(this, cam1_rgb);
    depth1_sub_.subscribe(this, cam1_depth);
    rgb2_sub_.subscribe(this, cam2_rgb);
    depth2_sub_.subscribe(this, cam2_depth);

    // Queue depth 10 with ApproximateTime, matching the original ROS1
    // ros_rgbd.cc synchronizer policy exactly.
    sync_ = std::make_shared<Synchronizer>(
      SyncPolicy(10), rgb1_sub_, depth1_sub_, rgb2_sub_, depth2_sub_);
    sync_->registerCallback(
      std::bind(
        &MulticamSlamNode::imageCallback, this,
        std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4));

    RCLCPP_INFO(
      this->get_logger(),
      "multicam_slam_node ready: subscribed to '%s', '%s', '%s', '%s', "
      "publishing '/multicam_slam/odom'",
      cam1_rgb.c_str(), cam1_depth.c_str(), cam2_rgb.c_str(), cam2_depth.c_str());
  }

  ~MulticamSlamNode() override
  {
    if (slam_system_) {
      RCLCPP_INFO(this->get_logger(), "Shutting down Multi_ORB_SLAM system...");
      slam_system_->Shutdown();
    }
  }

private:
  static bool toCvShare(
    const ImageMsg::ConstSharedPtr & msg, cv_bridge::CvImageConstPtr & out,
    const rclcpp::Logger & logger)
  {
    try {
      out = cv_bridge::toCvShare(msg);
      return true;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(logger, "cv_bridge conversion failed: %s", e.what());
      return false;
    }
  }

  void imageCallback(
    const ImageMsg::ConstSharedPtr & msgRGB1, const ImageMsg::ConstSharedPtr & msgD1,
    const ImageMsg::ConstSharedPtr & msgRGB2, const ImageMsg::ConstSharedPtr & msgD2)
  {
    cv_bridge::CvImageConstPtr cvRGB1, cvD1, cvRGB2, cvD2;
    const auto & logger = this->get_logger();
    if (!toCvShare(msgRGB1, cvRGB1, logger) || !toCvShare(msgD1, cvD1, logger) ||
      !toCvShare(msgRGB2, cvRGB2, logger) || !toCvShare(msgD2, cvD2, logger))
    {
      return;
    }

    const double timestamp =
      static_cast<double>(msgRGB1->header.stamp.sec) +
      static_cast<double>(msgRGB1->header.stamp.nanosec) * 1e-9;

    // TrackRGBD's return (Tcw) was previously only ever written to a
    // TUM-format trajectory file on SLAM.Shutdown() in the ROS1 wrapper --
    // it was never published live. Publishing it per-frame as Odometry
    // below is new functionality added by this wrapper.
    const cv::Mat Tcw = slam_system_->TrackRGBD(
      cvRGB1->image, cvD1->image, cvRGB2->image, cvD2->image, timestamp);

    publishOdometry(Tcw, msgRGB1->header.stamp);
  }

  void publishOdometry(const cv::Mat & Tcw, const builtin_interfaces::msg::Time & stamp)
  {
    // Tracking lost / not yet initialized: TrackRGBD() returns an empty
    // cv::Mat in that case. Skip publishing rather than emit a bogus pose.
    if (Tcw.empty()) {
      return;
    }

    // ORB_SLAM2::System::TrackRGBD returns Tcw (world-to-camera, 4x4,
    // CV_32F). Invert to get Twc (camera pose in the world frame) for
    // odometry, matching convention.
    const cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
    const cv::Mat Rwc = Rcw.t();
    const cv::Mat twc = -Rwc * tcw;

    Eigen::Matrix3f R;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        R(i, j) = Rwc.at<float>(i, j);
      }
    }
    const Eigen::Quaternionf q(R);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "camera";

    odom_msg.pose.pose.position.x = twc.at<float>(0);
    odom_msg.pose.pose.position.y = twc.at<float>(1);
    odom_msg.pose.pose.position.z = twc.at<float>(2);

    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_pub_->publish(odom_msg);
  }

  std::unique_ptr<ORB_SLAM2::System> slam_system_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  message_filters::Subscriber<ImageMsg> rgb1_sub_;
  message_filters::Subscriber<ImageMsg> depth1_sub_;
  message_filters::Subscriber<ImageMsg> rgb2_sub_;
  message_filters::Subscriber<ImageMsg> depth2_sub_;
  std::shared_ptr<Synchronizer> sync_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<MulticamSlamNode>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
