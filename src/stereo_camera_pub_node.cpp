#include <iostream>
#include "orb_slam2davinchi/stereo_camera_pub_node.hpp"

#define VIDEO_PATH "/home/bkmn/colcon_ws/video/20200218_102324_0_0.avi"

stereo_camera_pub_node::stereo_camera_pub_node(
    const std::string &node_name,
    const rclcpp::NodeOptions &node_options)
    : Node(node_name, node_options),
      image_transport_(nullptr),
      node_name_(node_name)
{
    declare_parameter("video_path", VIDEO_PATH);
    declare_parameter("enable_camera", rclcpp::ParameterValue(false));
    declare_parameter("camera_num", rclcpp::ParameterValue(0));

}

void stereo_camera_pub_node::init()
{

    get_parameter("video_path", video_path_);
    get_parameter("enable_camera", enable_camera_);
    get_parameter("camera_num", camera_num_);

    if(enable_camera_){
        video_.open(camera_num_);
        rclcpp::shutdown();
    }

    video_.open(video_path_);
    if(!video_.isOpened()){
        RCLCPP_ERROR(this->get_logger(), "Can't open video");
        //throw std::runtime_error("Can't open video");
    }

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;

    height_ = video_.get(cv::CAP_PROP_FRAME_HEIGHT);
    width_ = video_.get(cv::CAP_PROP_FRAME_WIDTH);
    fps_ = video_.get(cv::CAP_PROP_FPS);
    fps_count_ = video_.get(cv::CAP_PROP_FRAME_COUNT);
    image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    left_image_pub_ = image_transport_->advertise(node_name_ + "/left/image_raw", 10);
    right_image_pub_ = image_transport_->advertise(node_name_ + "/right/image_raw", 10);
    //left_image_pub1_ = this->create_publisher<sensor_msgs::msg::Image>(node_name_ + "/left/image_raw", rclcpp::QoS(10));
    //right_image_pub1_ = this->create_publisher<sensor_msgs::msg::Image>(node_name_ + "/right/image_raw", rclcpp::QoS(10));
    pubsize_ = cv::Size(uint(width_/2), height_);
    left_image_start_point = cv::Point(0,0);
    right_image_start_point = cv::Point(uint(width_/2), 0);

    left_image_roi = cv::Rect(left_image_start_point, pubsize_);
    right_image_roi = cv::Rect(right_image_start_point, pubsize_);

    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(node_name_ + "/camera_info", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(33),
                                     std::bind(&stereo_camera_pub_node::TimerCallback, this));
    
    timer_camera_info_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                                 std::bind(&stereo_camera_pub_node::TimerCallback2_, this));

    
    camera_info.header.frame_id = "davinci";
    camera_info.height = height_;
    camera_info.width = width_;
    
    
    camera_info.k[0] = 5.2569061657511509e+02;
    camera_info.k[4] = 5.2569061657511509e+02;
    camera_info.k[2] = 4.6150557447981282e+02;
    camera_info.k[5] = 2.7409307532556096e+02;

    camera_info.d.resize(9);
    camera_info.d[0] = -3.4828776048595911e-02;
    camera_info.d[1] = 5.3255842474875789e-02;
    camera_info.d[2] = 0.0;
    camera_info.d[3] = 0;
    camera_info.d[4] = 0;
    camera_info.p[3] = 0;
    
    RCLCPP_INFO(this->get_logger(), "Create instance");

}

void stereo_camera_pub_node::publish_left_camera(cv::Mat image)
{
    std_msgs::msg::Header header;
    header.stamp = current_frame_time_;
    header.frame_id = "sss";
    const sensor_msgs::msg::Image::SharedPtr image_msg =
        cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    this->left_image_pub_.publish(image_msg);
    //this->left_image_pub1_->publish(*image_msg);
}

void stereo_camera_pub_node::publish_right_camera(cv::Mat image)
{
    std_msgs::msg::Header header;
    header.stamp = current_frame_time_;
    header.frame_id = "sss";
    const sensor_msgs::msg::Image::SharedPtr image_msg =
        cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    this->right_image_pub_.publish(image_msg);
    //this->right_image_pub1_->publish(*image_msg);
}

void stereo_camera_pub_node::TimerCallback()
{
    video_ >> frame_;

    if(frame_.empty()){
        RCLCPP_ERROR(this->get_logger(), "No Frame in video");
        rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "pub");
    cv::Mat left_image = frame_(left_image_roi);
    cv::Mat right_image = frame_(right_image_roi);

    current_frame_time_ = rclcpp::Time();
    
    //this->camera_info_pub_->publish(camera_info);
    
    publish_left_camera(left_image);
    publish_right_camera(right_image);
    
}

void stereo_camera_pub_node::TimerCallback2_()
{
    camera_info_pub_->publish(camera_info);
}