// Stereo-Inertial node
#ifndef STEREO_INERTIAL_HPP
#define STEREO_INERTIAL_HPP

#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <queue>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "System.h"
#include "ImuTypes.h"

class ImuGrabber : public rclcpp::Node
{
public:
    ImuGrabber() : Node("stereo_inertial_node") {}
    
    void initializeImuSubscription(const std::string &imu_topic)
    {
        rclcpp::QoS imu_qos(500);
        imu_qos.best_effort();
        imu_qos.durability_volatile();
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, imu_qos,
            std::bind(&ImuGrabber::GrabImu, this, std::placeholders::_1)
        );
    }
    
    void GrabImu(const sensor_msgs::msg::Imu::ConstPtr &imu_msg);

    std::queue<sensor_msgs::msg::Imu::ConstPtr> imuBuf;
    std::mutex ImuBufMutex;

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
};

class ImageGrabber : public ImuGrabber
{
public:
    ImageGrabber();

    void GrabImageLeft(const sensor_msgs::msg::Image::ConstPtr& msg);
    void GrabImageRight(const sensor_msgs::msg::Image::ConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::msg::Image::ConstPtr &img_msg);
    void SyncWithImu();
    void publishOdomAndTf(const Sophus::SE3f& Twc, const rclcpp::Time& stamp);
    
    std::queue<sensor_msgs::msg::Image::ConstPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft, mBufMutexRight;
   
    // SLAM system
    std::unique_ptr<ORB_SLAM3::System> slam_system_;

    // Rectification
    bool do_rectify = false;
    cv::Mat M1l, M2l, M1r, M2r;

    // CLAHE
    bool mbClahe = false;
    cv::Ptr<cv::CLAHE> mClahe;

    // Output config
    std::string odomFrameId_ = "map";
    std::string baseFrameId_ = "camera_link";
    bool publishTf_ = true;

private:
    void initializeFromParameters();
    void setupRectificationMaps(const std::string &settings_yaml_path);

    // Topics
    std::string leftImageTopic_ = "/camera/camera/infra1/image_rect_raw";
    std::string rightImageTopic_ = "/camera/camera/infra2/image_rect_raw";
    std::string imuTopic_ = "/camera/camera/imu";

    // Parameters
    std::string vocFilePath_;
    std::string settingsDir_;
    std::string settingsName_;

    // Timing/robustness controls
    int minImuSamples_ = 0;            // minimum IMU samples before processing a frame
    int waitForImuMs_ = 0;             // bounded wait in ms to gather IMU for current frame
    bool dropOnTimeJump_ = false;      // drop frames with non-monotonic timestamps
    double minParallaxDeg_ = 1.0;      // minimum parallax for initialization (degrees)
    int minTrackingPoints_ = 100;      // minimum tracked points for stable tracking
    double lastImageTime_ = -1.0;      // last processed image timestamp

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_left;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_right;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
};

#endif


