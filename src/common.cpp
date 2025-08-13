/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/common.hpp"

//* Constructor
MonocularMode::MonocularMode() :Node("mono_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    homeDir = getenv("HOME");
    // std::cout<<"Home: "<<homeDir<<std::endl;
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings directory (Monocular)
    // Output related parameters
    this->declare_parameter("odom_topic", "/orb_slam3/odometry");
    this->declare_parameter("odom_frame_id", "map");
    this->declare_parameter("base_frame_id", "camera_link");
    this->declare_parameter("publish_tf", true);
    
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsDirPath = "file_not_set";
    settingsFilePath = "";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsDirPath = param3.as_string();

    // Output params
    rclcpp::Parameter p_odom_topic = this->get_parameter("odom_topic");
    rclcpp::Parameter p_odom_frame = this->get_parameter("odom_frame_id");
    rclcpp::Parameter p_base_frame = this->get_parameter("base_frame_id");
    rclcpp::Parameter p_publish_tf = this->get_parameter("publish_tf");
    odom_topic_ = p_odom_topic.as_string();
    odom_frame_id_ = p_odom_frame.as_string();
    base_frame_id_ = p_base_frame.as_string();
    publish_tf_ = p_publish_tf.as_bool();

    // rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
    
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsDirPath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsDirPath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }

    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    // RCLCPP_INFO(this->get_logger(), "settings_dir %s", settingsDirPath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp ndoe
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive RGB image messages
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; // topic to receive RGB image messages

    //* subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    //* subscrbite to the image messages coming from the Python driver node
    subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));

    //* subscribe to receive the timestep
    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));

    //* publishers for outputs
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
    
}

//* Destructor
MonocularMode::~MonocularMode()
{   
    // Release resources and cleanly shutdown
    if (pAgent != nullptr) {
        try {
            pAgent->Shutdown();
        } catch (...) {}
        delete pAgent;
        pAgent = nullptr;
    }
    pass;

}

//* Callback which accepts experiment parameters from the Python node
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg){
    
    // std::cout<<"experimentSetting_callback"<<std::endl;
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    receivedConfig = experimentConfig;
    // receivedConfig = experimentConfig; // Redundant
    
    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->receivedConfig.c_str());

    //* Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";
    
    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    configAck_publisher_->publish(message);

    //* Initialize VSLAM once
    if (!vslam_initialized_)
    {
        initializeVSLAM(experimentConfig);
    }

}

//* Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string& configString){
    
    // Watchdog, if the paths to vocabulary and settings directory are still not set
    if (vocFilePath == "file_not_set" || settingsDirPath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    //* Build .yaml`s file path
    settingsFilePath = settingsDirPath;
    settingsFilePath.append(configString);
    settingsFilePath.append(".yaml"); // Example .../Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::MONOCULAR; 
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    try {
        pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
        vslam_initialized_ = true;
        std::cout << "MonocularMode node initialized" << std::endl; // TODO needs a better message
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize ORB-SLAM3 System: %s", e.what());
        rclcpp::shutdown();
        return;
    }
}

//* Callback that processes timestep sent over ROS
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    // timeStep = 0; // Initialize
    timeStep = time_msg.data;
}

//* Callback to process image message and run SLAM node
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
        
        // DEBUGGING, Show image
        // Update GUI Window
        // cv::imshow("test_window", cv_ptr->image);
        // cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    
    // std::cout<<std::fixed<<"Timestep: "<<timeStep<<std::endl; // Debug
    
    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    if (pAgent == nullptr) {
        RCLCPP_WARN(this->get_logger(), "Track called before initialization. Dropping frame.");
        return;
    }
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep); 

    // Publish odometry and TF using Twc (pose of camera in world/map frame)
    Sophus::SE3f Twc = Tcw.inverse();

    // Translation
    const Eigen::Vector3f t = Twc.translation();
    // Rotation
    const Eigen::Matrix3f R = Twc.so3().matrix();
    Eigen::Quaternionf q(R);
    q.normalize();

    // Timestamp
    rclcpp::Time stamp = this->now();

    // Odometry message
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
    // Leave twist zero (no velocity estimation exposed here)
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


