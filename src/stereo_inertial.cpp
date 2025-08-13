#include "ros2_orb_slam3/stereo_inertial.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <opencv2/core/persistence.hpp>

ImageGrabber::ImageGrabber()
{
    // Declare parameters with defaults
    this->declare_parameter<std::string>("voc_file_arg", "");
    this->declare_parameter<std::string>("settings_file_path_arg", "");
    this->declare_parameter<std::string>("settings_name", "RealSense_D455");
    this->declare_parameter<std::string>("left_image_topic", "/camera/camera/infra1/image_rect_raw");
    this->declare_parameter<std::string>("right_image_topic", "/camera/camera/infra2/image_rect_raw");
    this->declare_parameter<std::string>("imu_topic", "/camera/camera/imu");
    this->declare_parameter<std::string>("odom_topic", "/orb_slam3/odometry");
    this->declare_parameter<std::string>("odom_frame_id", "map");
    this->declare_parameter<std::string>("base_frame_id", "camera_link");
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<bool>("do_rectify", false);
    this->declare_parameter<bool>("use_clahe", false);
    this->declare_parameter<int>("min_imu_samples", 0);
    this->declare_parameter<int>("wait_for_imu_ms", 0);
    this->declare_parameter<bool>("drop_on_time_jump", false);
    this->declare_parameter<double>("min_parallax_deg", 1.0);
    this->declare_parameter<int>("min_tracking_points", 100);
    // No strict IMU/timestamp gates by default

    initializeFromParameters();

    // QoS
    rclcpp::QoS image_qos(1);
    image_qos.best_effort();
    image_qos.durability_volatile();

    // Subscriptions
    sub_img_left = this->create_subscription<sensor_msgs::msg::Image>(
        leftImageTopic_, image_qos,
        std::bind(&ImageGrabber::GrabImageLeft, this, std::placeholders::_1)
    );
    sub_img_right = this->create_subscription<sensor_msgs::msg::Image>(
        rightImageTopic_, image_qos,
        std::bind(&ImageGrabber::GrabImageRight, this, std::placeholders::_1)
    );

    // IMU subscription
    this->initializeImuSubscription(imuTopic_);

    // Publishers
    odomPublisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        this->get_parameter("odom_topic").as_string(), 10);
    tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Rectification maps
    if (do_rectify) {
        const std::string settings_yaml_path = settingsDir_ + "/" + settingsName_ + ".yaml";
        setupRectificationMaps(settings_yaml_path);
    }
}

void ImageGrabber::initializeFromParameters()
{
    vocFilePath_ = this->get_parameter("voc_file_arg").as_string();
    settingsDir_ = this->get_parameter("settings_file_path_arg").as_string();
    settingsName_ = this->get_parameter("settings_name").as_string();
    leftImageTopic_ = this->get_parameter("left_image_topic").as_string();
    rightImageTopic_ = this->get_parameter("right_image_topic").as_string();
    imuTopic_ = this->get_parameter("imu_topic").as_string();
    odomFrameId_ = this->get_parameter("odom_frame_id").as_string();
    baseFrameId_ = this->get_parameter("base_frame_id").as_string();
    publishTf_ = this->get_parameter("publish_tf").as_bool();
    do_rectify = this->get_parameter("do_rectify").as_bool();
    mbClahe = this->get_parameter("use_clahe").as_bool();
    minImuSamples_ = this->get_parameter("min_imu_samples").as_int();
    waitForImuMs_ = this->get_parameter("wait_for_imu_ms").as_int();
    dropOnTimeJump_ = this->get_parameter("drop_on_time_jump").as_bool();
    minParallaxDeg_ = this->get_parameter("min_parallax_deg").as_double();
    minTrackingPoints_ = this->get_parameter("min_tracking_points").as_int();
    // No strict IMU/timestamp gates by default

    if (mbClahe) {
        mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    }

    // Start ORB-SLAM3 system (IMU stereo, viewer=true)
    const std::string settings_yaml_path = settingsDir_.empty() ? "" : (settingsDir_ + "/" + settingsName_ + ".yaml");
    if (vocFilePath_.empty() || settings_yaml_path.empty()) {
        RCLCPP_FATAL(this->get_logger(), "Missing required parameters: voc_file_arg or settings_file_path_arg/settings_name");
        throw std::runtime_error("Missing required parameters");
    }
    slam_system_ = std::make_unique<ORB_SLAM3::System>(
        vocFilePath_, settings_yaml_path, ORB_SLAM3::System::IMU_STEREO, true);
}

void ImageGrabber::setupRectificationMaps(const std::string &settings_yaml_path)
{
    cv::FileStorage fsSettings(settings_yaml_path, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        RCLCPP_FATAL(this->get_logger(), "ERROR: Wrong path to settings: %s", settings_yaml_path.c_str());
        throw std::runtime_error("Wrong path to settings");
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;
    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;
    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;
    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = (int)fsSettings["LEFT.height"];
    int cols_l = (int)fsSettings["LEFT.width"];
    int rows_r = (int)fsSettings["RIGHT.height"];
    int cols_r = (int)fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
    {
        RCLCPP_FATAL(this->get_logger(), "ERROR: Calibration parameters to rectify stereo are missing!");
        throw std::runtime_error("Missing rectification parameters");
    }

    cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0,3).colRange(0,3), cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
    cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0,3).colRange(0,3), cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);
}

void ImuGrabber::GrabImu(const sensor_msgs::msg::Imu::ConstPtr &imu_msg)
{
    ImuBufMutex.lock();
    imuBuf.push(imu_msg);
    ImuBufMutex.unlock();
    return;
}

void ImageGrabber::GrabImageLeft(const sensor_msgs::msg::Image::ConstPtr &img_msg)
{
    mBufMutexLeft.lock();
    if (!imgLeftBuf.empty())
        imgLeftBuf.pop();
    imgLeftBuf.push(img_msg);
    mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::msg::Image::ConstPtr &img_msg)
{
    mBufMutexRight.lock();
    if (!imgRightBuf.empty())
        imgRightBuf.pop();
    imgRightBuf.push(img_msg);
    mBufMutexRight.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::msg::Image::ConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    
    if(cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void ImageGrabber::SyncWithImu()
{
    const double maxTimeDiff = 0.01;
    while(1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf.empty() && !imgRightBuf.empty())
        {
            tImLeft = imgLeftBuf.front()->header.stamp.sec + imgLeftBuf.front()->header.stamp.nanosec * 1e-9;
            tImRight = imgRightBuf.front()->header.stamp.sec + imgRightBuf.front()->header.stamp.nanosec * 1e-9;

            // No timestamp drop guard (restore original behavior)

            this->mBufMutexRight.lock();
            while((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1)
            {
                imgRightBuf.pop();
                tImRight = imgRightBuf.front()->header.stamp.sec + imgRightBuf.front()->header.stamp.nanosec * 1e-9;
            }
            this->mBufMutexRight.unlock();

            this->mBufMutexLeft.lock();
            while((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1)
            {
                imgLeftBuf.pop();
                tImLeft = imgLeftBuf.front()->header.stamp.sec + imgLeftBuf.front()->header.stamp.nanosec * 1e-9;
            }
            this->mBufMutexLeft.unlock();

            if((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
            {
                // std::cout << "big time difference" << std::endl;
                continue;
            }
            // Collect IMU up to image time once and proceed (original behavior)
            std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
            {
                ImuBufMutex.lock();
                while(!imuBuf.empty())
                {
                    double t = imuBuf.front()->header.stamp.sec + imuBuf.front()->header.stamp.nanosec * 1e-9;
                    if(t <= tImLeft)
                    {
                        cv::Point3f acc(imuBuf.front()->linear_acceleration.x, imuBuf.front()->linear_acceleration.y, imuBuf.front()->linear_acceleration.z);
                        cv::Point3f gyr(imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y, imuBuf.front()->angular_velocity.z);
                        vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                        imuBuf.pop();
                    }
                    else
                    {
                        break;
                    }
                }
                ImuBufMutex.unlock();
            }

            this->mBufMutexLeft.lock();
            // Capture output timestamp before popping the left image from queue
            uint32_t out_sec = imgLeftBuf.front()->header.stamp.sec;
            uint32_t out_nsec = imgLeftBuf.front()->header.stamp.nanosec;
            imLeft = GetImage(imgLeftBuf.front());
            imgLeftBuf.pop();
            this->mBufMutexLeft.unlock();

            this->mBufMutexRight.lock();
            imRight = GetImage(imgRightBuf.front());
            imgRightBuf.pop();
            this->mBufMutexRight.unlock();


            
            if(mbClahe)
            {
                mClahe->apply(imLeft, imLeft);
                mClahe->apply(imRight, imRight);
            }

            if(do_rectify)
            {
                cv::remap(imLeft, imLeft, M1l, M2l, cv::INTER_LINEAR);
                cv::remap(imRight, imRight, M1r, M2r, cv::INTER_LINEAR);
            }

            // Call TrackStereo with the collected IMU measurements
            Sophus::SE3f Tcw = slam_system_->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);
            Sophus::SE3f Twc = Tcw.inverse();
            // Use captured image timestamp for outputs to avoid jitter/lag
            rclcpp::Time img_stamp(out_sec, out_nsec, RCL_ROS_TIME);
            publishOdomAndTf(Twc, img_stamp);
            lastImageTime_ = tImLeft;

            // No explicit sleep; let ROS spin handle pacing.
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}
// No helper necessary in reverted behavior

void ImageGrabber::publishOdomAndTf(const Sophus::SE3f& Twc, const rclcpp::Time& stamp)
{
    const Eigen::Vector3f t = Twc.translation();
    const Eigen::Matrix3f R = Twc.so3().matrix();
    Eigen::Quaternionf q(R); 
    q.normalize();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odomFrameId_;
    odom.child_frame_id = baseFrameId_;
    odom.pose.pose.position.x = t.x();
    odom.pose.pose.position.y = t.y();
    odom.pose.pose.position.z = t.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odomPublisher_->publish(odom);

    if (publishTf_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = odomFrameId_;
        tf.child_frame_id = baseFrameId_;
        tf.transform.translation.x = t.x();
        tf.transform.translation.y = t.y();
        tf.transform.translation.z = t.z();
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        tfBroadcaster_->sendTransform(tf);
    }
}


