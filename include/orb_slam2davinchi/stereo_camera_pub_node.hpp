#ifndef __STEREO_CAMERA_PUB_NODE_HPP__
#define __STEREO_CAMERA_PUB_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>

#include <memory>
#include <filesystem>
#include <iterator>

#include "orb_slam2davinchi/image_processing.hpp"

class stereo_camera_pub_node : public rclcpp::Node
{
public:
    explicit stereo_camera_pub_node(const std::string &node_name="",
        const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
    ~stereo_camera_pub_node(){};
    void init();


private:
    void publish_left_camera(cv::Mat image);
    void publish_right_camera(cv::Mat image);
    void publish_left_mask(cv::Mat image);
    void publish_right_mask(cv::Mat image);
    void VideoPublishTimerCallback();
    void ImagePublishTimerCallback();
    void TimerCallback2CameraInfo();
    void SetTimeStamp();
    
    rclcpp::Time current_frame_time_;
    image_transport::Publisher left_image_pub_;
    image_transport::Publisher right_image_pub_;
    image_transport::Publisher left_mask_pub_;
    image_transport::Publisher right_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub1_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub1_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_image_pub_;
    rclcpp::TimerBase::SharedPtr timer_camera_info_;
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    sensor_msgs::msg::CameraInfo camera_info;

    // Args
    std::string video_path_;
    bool enable_camera_;
    bool pub_from_image_;
    std::string image_directory_;
    int camera_num_;
    bool is_grayscale_;
    bool is_params_file_;
    std::string params_file_path_;
    bool is_pub_mask_;
    std::string mask_video_path_;
    std::string image_proc_param_;
    
    ImageProcessing img_proc_;

    cv::VideoCapture video_;
    cv::VideoCapture mask_video_;
    std::vector<std::string> image_paths_;
    std::vector<std::string>::iterator image_path_itr_;

    std::string frame_id_;
    cv::Mat frame_;
    cv::Mat frame_mask_;
    uint32_t fps_;
    uint32_t frame_count_;
    uint32_t width_;
    uint32_t height_;
    uint32_t pub_width_;
    uint32_t pub_height_;
    cv::Size pubsize_;
    cv::Point left_image_start_point;
    cv::Point right_image_start_point;
    cv::Rect left_image_roi;
    cv::Rect right_image_roi;

    // Camera Info
    double Camera_fx_;
    double Camera_fy_;
    double Camera_cx_;
    double Camera_cy_;
    double Camera_k1_;
    double Camera_k2_;
    double Camera_p1_;
    double Camera_p2_;
    double Camera_p3_;

    std::string node_name_;
};

#endif
