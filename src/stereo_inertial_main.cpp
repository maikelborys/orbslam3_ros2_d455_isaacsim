#include "ros2_orb_slam3/stereo_inertial.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ImageGrabber>();
    std::thread sync_thread(&ImageGrabber::SyncWithImu, node);
    rclcpp::spin(node);
    return 0;
}


