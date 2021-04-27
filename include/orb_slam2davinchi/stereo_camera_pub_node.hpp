#ifndef __STEREO_CAMERA_PUB_NODE_HPP__
#define __STEREO_CAMERA_PUB_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>

#include <memory>

class stereo_camera_pub_node : public rclcpp::Node
{
public:
    explicit stereo_camera_pub_node(const std::string &node_name="",
    const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
    ~stereo_camera_pub_node(){};

private:
    void publish_left_camera(cv::Mat image);
    void publish_right_camera(cv::Mat image);
    void TimerCallback();

    rclcpp::Time current_frame_time_;
    image_transport::Publisher left_image_pub_;
    image_transport::Publisher right_image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<image_transport::ImageTransport> image_transport_;

    cv::VideoCapture video_;
    size_t frame_id_;
    cv::Mat frame_;
    uint32_t fps_;
    uint32_t fps_count_;
    uint32_t width_;
    uint32_t height_;
    uint32_t pub_width_;
    uint32_t pub_height_;
    cv::Size pubsize_;
    cv::Point left_image_start_point;
    cv::Point right_image_start_point;
    cv::Rect left_image_roi;
    cv::Rect right_image_roi;


    std::string node_name_;
};

#endif
