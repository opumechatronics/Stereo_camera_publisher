#include <iostream>
#include "orb_slam2davinchi/stereo_camera_pub_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<stereo_camera_pub_node>("stereo_camera_publisher");
    node->init();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;

}