#include "ros2_orb_slam3/stereo.hpp"
#include "MapPoint.h"
#include <chrono>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <regex>

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
  this->declare_parameter("start_localization_only", false);
  this->declare_parameter("load_atlas_basename", "");
  this->declare_parameter("maps_dir", std::string("/home/robot/ros2_test/src/ros2_orb_slam3/maps"));

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
  start_localization_only_ = this->get_parameter("start_localization_only").as_bool();
  const std::string load_atlas_basename_param = this->get_parameter("load_atlas_basename").as_string();
  const std::string maps_dir = this->get_parameter("maps_dir").as_string();

  if (vocFilePath == "file_not_set" || settingsDirPath == "file_not_set") {
    vocFilePath = homeDir + std::string("/") + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
    settingsDirPath = homeDir + std::string("/") + packagePath + "orb_slam3/config/Stereo/";
  }

  RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
  RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());

  // Configure ORB_SLAM3_LOAD_ATLAS if requested and prepare patched YAML
  if (start_localization_only_) {
    std::string env_base;
    if (!load_atlas_basename_param.empty()) {
      env_base = load_atlas_basename_param;
    } else {
      const char* pre_env = std::getenv("ORB_SLAM3_LOAD_ATLAS");
      if (pre_env != nullptr) {
        env_base = std::string(pre_env);
      } else {
        // Auto-pick newest .osa from maps_dir
        try {
          std::filesystem::path newest;
          std::filesystem::file_time_type newest_time{};
          for (const auto& entry : std::filesystem::directory_iterator(maps_dir)) {
            if (!entry.is_regular_file()) continue;
            const auto& p = entry.path();
            if (p.extension() == ".osa") {
              auto t = std::filesystem::last_write_time(p);
              if (newest.empty() || t > newest_time) { newest = p; newest_time = t; }
            }
          }
          if (!newest.empty()) {
            env_base = newest.replace_extension("").string();
          }
        } catch (...) {}
      }
    }
    if (!env_base.empty()) {
      // Normalize: strip trailing .osa if present
      if (env_base.size() > 4 && env_base.substr(env_base.size()-4) == ".osa") {
        env_base = env_base.substr(0, env_base.size()-4);
      }
      setenv("ORB_SLAM3_LOAD_ATLAS", env_base.c_str(), 1);
      RCLCPP_INFO(this->get_logger(), "Set ORB_SLAM3_LOAD_ATLAS = %s", env_base.c_str());
      selected_atlas_basename_ = env_base;
    } else {
      RCLCPP_WARN(this->get_logger(), "start_localization_only requested but no atlas basename found; proceeding without preload");
    }
  }

  // If in localization-only mode with a selected atlas, copy the .osa to CWD and
  // create a temp YAML that sets System.LoadAtlasFromFile to the basename
  if (start_localization_only_ && !selected_atlas_basename_.empty()) {
    try {
      std::filesystem::path src = std::filesystem::path(selected_atlas_basename_ + ".osa");
      std::string base_only = std::filesystem::path(selected_atlas_basename_).filename().string();
      std::filesystem::path dst = std::filesystem::current_path() / (base_only + ".osa");
      if (!std::filesystem::exists(dst)) {
        std::filesystem::copy_file(src, dst, std::filesystem::copy_options::overwrite_existing);
      }

      // Build original settings path and read it
      std::filesystem::path orig_yaml = std::filesystem::path(settingsDirPath) / (settings_name + ".yaml");
      std::ifstream in(orig_yaml);
      std::stringstream buffer;
      buffer << in.rdbuf();
      std::string content = buffer.str();

      // Replace or append System.LoadAtlasFromFile
      std::istringstream iss(content);
      std::ostringstream oss;
      std::string line;
      bool replaced = false;
      while (std::getline(iss, line)) {
        std::string trimmed = line;
        // trim leading spaces
        trimmed.erase(trimmed.begin(), std::find_if(trimmed.begin(), trimmed.end(), [](unsigned char ch){ return !std::isspace(ch); }));
        if (!trimmed.empty() && trimmed[0] != '#') {
          const std::string key = "System.LoadAtlasFromFile:";
          auto pos = trimmed.find(key);
          if (pos == 0) {
            oss << "System.LoadAtlasFromFile: \"" << base_only << "\"\n";
            replaced = true;
            continue;
          }
        }
        oss << line << '\n';
      }
      if (!replaced) {
        oss << "\n# Added for localization\nSystem.LoadAtlasFromFile: \"" << base_only << "\"\n";
      }

      // Write temp YAML
      std::filesystem::path tmp = std::filesystem::temp_directory_path() / (std::string("orbslam3_localize_") + std::to_string(::getpid()) + ".yaml");
      std::ofstream out(tmp);
      out << oss.str();
      out.close();
      temp_settings_path_ = tmp.string();
      RCLCPP_INFO(this->get_logger(), "Patched settings for localization: %s", temp_settings_path_.c_str());
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Failed to prepare atlas/YAML for localization: %s", e.what());
    }
  }

  // Initialize VSLAM immediately with settings_name (or patched temp YAML)
  initializeVSLAM(settings_name);

  // Synchronized stereo subscriptions (SensorDataQoS)
  left_sub_ = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, left_image_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
  right_sub_ = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, right_image_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
  sync_ = std::make_shared<message_filters::Synchronizer<ApproxPolicy>>(ApproxPolicy(10), *left_sub_, *right_sub_);
  sync_->registerCallback(std::bind(&StereoMode::GrabStereo, this, std::placeholders::_1, std::placeholders::_2));

  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
  viz_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(viz_topic_, 1);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Initialize path message
  path_msg_.header.frame_id = odom_frame_id_;
  path_msg_.poses.clear();

  // Start 1 Hz visualization timer
  viz_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&StereoMode::publishVisualization, this));

  // Initialize services
  reset_button_service_ = this->create_service<std_srvs::srv::Trigger>(
      "press_reset_button",
      std::bind(&StereoMode::handlePressResetButton, this, std::placeholders::_1, std::placeholders::_2));
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
  if (!temp_settings_path_.empty()) {
    settingsFilePath = temp_settings_path_;
  } else {
    settingsFilePath = settingsDirPath;
    settingsFilePath.append(configString);
    settingsFilePath.append(".yaml");
  }
  RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());

  sensorType = ORB_SLAM3::System::STEREO;
  try {
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    vslam_initialized_ = true;
    std::cout << "StereoMode node initialized" << std::endl;
    
    // Activate localization mode immediately after initialization if requested
    if (start_localization_only_) {
      pAgent->ActivateLocalizationMode();
      RCLCPP_INFO(this->get_logger(), "Localization mode activated (stereo)");
      RCLCPP_INFO(this->get_logger(), "Localization target atlas: %s", selected_atlas_basename_.c_str());
      
      // Note: Reset button needs to be pressed manually in the viewer
      // The localization mode button is now automatically checked
      RCLCPP_INFO(this->get_logger(), "Localization mode activated automatically. Please press RESET button in viewer if needed.");
    }
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

  // Determine tracking state to gate publishing
  int tracking_state = pAgent->GetTrackingState();
  const bool tracking_ok = (tracking_state == 2 /*Tracking::OK*/);
  ok_streak_ = tracking_ok ? std::min(ok_streak_ + 1, 1000000) : 0;

  // Detect first relocalization (first time tracking becomes OK)
  if (!relocalized_ && ok_streak_ >= required_ok_streak_) {
    relocalized_ = true;
    path_msg_.poses.clear();
    RCLCPP_INFO(this->get_logger(), "Relocalized (stable) on atlas; starting odom/path publishing");
  }

  if (!relocalized_) {
    // Suppress publishing until relocalized to avoid inconsistent path/odom
    return;
  }

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

  // Append to path
  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp = stamp;
  ps.header.frame_id = odom_frame_id_;
  ps.pose = odom_msg.pose.pose;
  path_msg_.header.stamp = stamp;
  path_msg_.poses.push_back(ps);
  path_publisher_->publish(path_msg_);

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
    // Only publish TF when tracking is OK to avoid jumps
    if (tracking_ok) {
      tf_broadcaster_->sendTransform(tf_msg);
    }
  }
}

void StereoMode::publishVisualization() {
  if (!vslam_initialized_ || pAgent == nullptr) return;
  visualization_msgs::msg::MarkerArray arr;

  // Landmarks as points
  visualization_msgs::msg::Marker pts;
  pts.header.frame_id = odom_frame_id_;
  pts.header.stamp = this->now();
  pts.ns = "map_points";
  pts.id = 0;
  pts.type = visualization_msgs::msg::Marker::POINTS;
  pts.action = visualization_msgs::msg::Marker::ADD;
  pts.scale.x = 0.02;
  pts.scale.y = 0.02;
  pts.color.a = 1.0;
  pts.color.r = 0.0f;
  pts.color.g = 1.0f;
  pts.color.b = 0.0f;

  const std::vector<ORB_SLAM3::MapPoint*> vMPs = pAgent->GetAllMapPoints();
  pts.points.reserve(vMPs.size());
  for (auto* mp : vMPs) {
    if (!mp) continue;
    const Eigen::Vector3f pos = mp->GetWorldPos();
    geometry_msgs::msg::Point p;
    p.x = pos.x();
    p.y = pos.y();
    p.z = pos.z();
    pts.points.push_back(p);
  }
  arr.markers.push_back(pts);

  // Keyframes as small axes (only positions as spheres for simplicity)
  visualization_msgs::msg::Marker kf;
  kf.header.frame_id = odom_frame_id_;
  kf.header.stamp = this->now();
  kf.ns = "keyframes";
  kf.id = 1;
  kf.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  kf.action = visualization_msgs::msg::Marker::ADD;
  kf.scale.x = 0.05;
  kf.scale.y = 0.05;
  kf.scale.z = 0.05;
  kf.color.a = 1.0;
  kf.color.r = 0.0f;
  kf.color.g = 0.5f;
  kf.color.b = 1.0f;
  const std::vector<Sophus::SE3f> vKFs = pAgent->GetAllKeyframePoses();
  kf.points.reserve(vKFs.size());
  for (const auto& Tcw_kf : vKFs) {
    const Sophus::SE3f Twc_kf = Tcw_kf.inverse();
    const Eigen::Vector3f t = Twc_kf.translation();
    geometry_msgs::msg::Point p;
    p.x = t.x(); p.y = t.y(); p.z = t.z();
    kf.points.push_back(p);
  }
  arr.markers.push_back(kf);

  viz_publisher_->publish(arr);
}

void StereoMode::handlePressResetButton(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (pAgent != nullptr) {
    RCLCPP_INFO(this->get_logger(), "Service called: Pressing RESET button programmatically");
    
    // Simulate pressing the reset button exactly like the viewer does
    // This includes resetting the active map AND deactivating localization mode
    if (start_localization_only_) {
      RCLCPP_INFO(this->get_logger(), "Deactivating localization mode before reset...");
      pAgent->DeactivateLocalizationMode();
    }
    
    // Reset the active map (this is what the viewer's reset button does)
    pAgent->ResetActiveMap();
    
    // If we were in localization mode, reactivate it after reset
    if (start_localization_only_) {
      RCLCPP_INFO(this->get_logger(), "Reactivating localization mode after reset...");
      pAgent->ActivateLocalizationMode();
    }
    
    response->success = true;
    response->message = "Reset button pressed successfully (with proper localization mode handling)";
    RCLCPP_INFO(this->get_logger(), "Reset button pressed successfully");
  } else {
    response->success = false;
    response->message = "ORB-SLAM3 system not initialized";
    RCLCPP_ERROR(this->get_logger(), "Cannot press reset button: ORB-SLAM3 system not initialized");
  }
}


