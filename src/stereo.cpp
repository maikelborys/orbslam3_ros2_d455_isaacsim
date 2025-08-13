#include "ros2_orb_slam3/stereo.hpp"

StereoMode::StereoMode() : Node("stereo_node_cpp") {
  homeDir = getenv("HOME");
  RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3 Stereo NODE STARTED");

  this->declare_parameter("node_name_arg", "not_given");
  this->declare_parameter("voc_file_arg", "file_not_set");
  this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // directory path
  this->declare_parameter("settings_name", "RealSense_D435i");
  this->declare_parameter("left_image_topic", "/camera/camera/infra1/image_rect_raw");
  this->declare_parameter("right_image_topic", "/camera/camera/infra2/image_rect_raw");
  this->declare_parameter("odom_topic", "/orb_slam3/odometry");
  this->declare_parameter("odom_frame_id", "map");
  this->declare_parameter("base_frame_id", "camera_link");
  this->declare_parameter("publish_tf", true);

  nodeName = this->get_parameter("node_name_arg").as_string();
  vocFilePath = this->get_parameter("voc_file_arg").as_string();
  settingsDirPath = this->get_parameter("settings_file_path_arg").as_string();
  std::string settings_name = this->get_parameter("settings_name").as_string();
  left_image_topic_ = this->get_parameter("left_image_topic").as_string();
  right_image_topic_ = this->get_parameter("right_image_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = this->get_parameter("base_frame_id").as_string();
  publish_tf_ = this->get_parameter("publish_tf").as_bool();

  if (vocFilePath == "file_not_set" || settingsDirPath == "file_not_set") {
    vocFilePath = homeDir + std::string("/") + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
    settingsDirPath = homeDir + std::string("/") + packagePath + "orb_slam3/config/Stereo/";
  }

  RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
  RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());

  // Initialize VSLAM immediately with settings_name
  initializeVSLAM(settings_name);

  // Synchronized stereo subscriptions (SensorDataQoS)
  left_sub_ = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, left_image_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
  right_sub_ = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, right_image_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
  sync_ = std::make_shared<message_filters::Synchronizer<ApproxPolicy>>(ApproxPolicy(10), *left_sub_, *right_sub_);
  sync_->registerCallback(std::bind(&StereoMode::GrabStereo, this, std::placeholders::_1, std::placeholders::_2));

  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

StereoMode::~StereoMode() {
  if (pAgent != nullptr) {
    try { pAgent->Shutdown(); } catch (...) {}
    delete pAgent; pAgent = nullptr;
  }
}

void StereoMode::initializeVSLAM(const std::string& configString) {
  if (vocFilePath == "file_not_set" || settingsDirPath == "file_not_set") {
    RCLCPP_ERROR(this->get_logger(), "Please provide valid voc_file and settings_dir paths");
    rclcpp::shutdown();
    return;
  }
  settingsFilePath = settingsDirPath;
  settingsFilePath.append(configString);
  settingsFilePath.append(".yaml");
  RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());

  sensorType = ORB_SLAM3::System::STEREO;
  try {
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    vslam_initialized_ = true;
    std::cout << "StereoMode node initialized" << std::endl;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize ORB-SLAM3 System: %s", e.what());
    rclcpp::shutdown();
  }
}

void StereoMode::GrabStereo(const ImageMsg::ConstSharedPtr left, const ImageMsg::ConstSharedPtr right) {
  if (!vslam_initialized_ || pAgent == nullptr) return;
  cv_bridge::CvImageConstPtr cv_left, cv_right;
  try { cv_left = cv_bridge::toCvShare(left); } catch (...) { RCLCPP_ERROR(this->get_logger(), "cv_bridge left err"); return; }
  try { cv_right = cv_bridge::toCvShare(right); } catch (...) { RCLCPP_ERROR(this->get_logger(), "cv_bridge right err"); return; }

  const double ts = static_cast<double>(left->header.stamp.sec) + static_cast<double>(left->header.stamp.nanosec) * 1e-9;
  Sophus::SE3f Tcw = pAgent->TrackStereo(cv_left->image, cv_right->image, ts);
  Sophus::SE3f Twc = Tcw.inverse();

  const Eigen::Vector3f t = Twc.translation();
  const Eigen::Matrix3f R = Twc.so3().matrix();
  Eigen::Quaternionf q(R); q.normalize();

  rclcpp::Time stamp = this->now();

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = odom_frame_id_;
  odom_msg.child_frame_id = base_frame_id_;
  odom_msg.pose.pose.position.x = static_cast<double>(t.x());
  odom_msg.pose.pose.position.y = static_cast<double>(t.y());
  odom_msg.pose.pose.position.z = static_cast<double>(t.z());
  odom_msg.pose.pose.orientation.x = static_cast<double>(q.x());
  odom_msg.pose.pose.orientation.y = static_cast<double>(q.y());
  odom_msg.pose.pose.orientation.z = static_cast<double>(q.z());
  odom_msg.pose.pose.orientation.w = static_cast<double>(q.w());
  odom_publisher_->publish(odom_msg);

  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = odom_frame_id_;
    tf_msg.child_frame_id = base_frame_id_;
    tf_msg.transform.translation.x = static_cast<double>(t.x());
    tf_msg.transform.translation.y = static_cast<double>(t.y());
    tf_msg.transform.translation.z = static_cast<double>(t.z());
    tf_msg.transform.rotation.x = static_cast<double>(q.x());
    tf_msg.transform.rotation.y = static_cast<double>(q.y());
    tf_msg.transform.rotation.z = static_cast<double>(q.z());
    tf_msg.transform.rotation.w = static_cast<double>(q.w());
    tf_broadcaster_->sendTransform(tf_msg);
  }
}


