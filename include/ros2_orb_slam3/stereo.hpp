// Include file 
#ifndef STEREO_HPP
#define STEREO_HPP

#include <iostream>
#include <mutex>
#include <string>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include "std_srvs/srv/trigger.hpp"

#include "System.h"

using std::placeholders::_1;

#define pass (void)0

class StereoMode : public rclcpp::Node {
  public:
    StereoMode();
    ~StereoMode();

  private:
    // Paths and params
    std::string homeDir = "";
    std::string packagePath = "ros2_test/src/ros2_orb_slam3/";
    std::string nodeName = "";
    std::string vocFilePath = "";
    std::string settingsDirPath = "";  // directory to Stereo YAMLs
    std::string settingsFilePath = ""; // full YAML path

    // ORB-SLAM3
    ORB_SLAM3::System* pAgent = nullptr;
    ORB_SLAM3::System::eSensor sensorType;
    bool vslam_initialized_ = false;
    bool enablePangolinWindow = true;
    bool enableOpenCVWindow = false;
    std::string selected_atlas_basename_ = "";

    // Topics
    std::string left_image_topic_ = "/camera/camera/infra1/image_rect_raw";
    std::string right_image_topic_ = "/camera/camera/infra2/image_rect_raw";

    // ROS I/O (synchronized stereo)
    using ImageMsg = sensor_msgs::msg::Image;
    using ApproxPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> left_sub_;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<ApproxPolicy>> sync_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Output config
    std::string odom_topic_ = "/orb_slam3/odometry";
    std::string path_topic_ = "/orb_slam3/path";
    std::string viz_topic_ = "/orb_slam3/markers";
    std::string odom_frame_id_ = "map";
    std::string base_frame_id_ = "camera_link";
    bool publish_tf_ = true;
    bool start_localization_only_ = false;

    // Path accumulation
    nav_msgs::msg::Path path_msg_;

    // Timer for visualization
    rclcpp::TimerBase::SharedPtr viz_timer_;
    void publishVisualization();

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_button_service_;
    void handlePressResetButton(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Relocalization guard
    bool relocalized_ = false;
    int ok_streak_ = 0;
    int required_ok_streak_ = 5;

    // Atlas selection
    std::string temp_settings_path_ = "";

    // Timing
    // Helpers / callbacks
    void initializeVSLAM(const std::string& settings_name);
    void GrabStereo(const ImageMsg::ConstSharedPtr left, const ImageMsg::ConstSharedPtr right);
};

#endif


