#include <iostream>
#include "orb_slam2davinchi/stereo_camera_pub_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<stereo_camera_pub_node>());

    rclcpp::shutdown();
    return 0;

}