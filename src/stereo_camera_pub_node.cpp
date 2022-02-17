#include <iostream>
#include <filesystem>
#include <vector>
#include <string>
#include "orb_slam2davinchi/stereo_camera_pub_node.hpp"

//#define VIDEO_PATH "/home/bkmn/colcon_ws/video/20200218_102324_0_0.avi"
#define VIDEO_PATH ""

// constructor
stereo_camera_pub_node::stereo_camera_pub_node(
    const std::string &node_name,
    const rclcpp::NodeOptions &node_options)
    : Node(node_name, node_options),
      image_transport_(nullptr),
      node_name_(node_name)
{
    declare_parameter("video_path", VIDEO_PATH);
    declare_parameter("enable_camera", rclcpp::ParameterValue(false));
    declare_parameter("pub_from_image", rclcpp::ParameterValue(false));
    declare_parameter("camera_num", rclcpp::ParameterValue(0));
    declare_parameter("image_path", "");
    declare_parameter("fps", rclcpp::ParameterValue(30));
    declare_parameter("frame_id", "davinci");
    declare_parameter("is_grayscale", rclcpp::ParameterValue(false));

    declare_parameter("Camera_fx", rclcpp::ParameterValue(double(0.0)));
    declare_parameter("Camera_fy", rclcpp::ParameterValue(double(0.0)));
    declare_parameter("Camera_cx", rclcpp::ParameterValue(double(0.0)));
    declare_parameter("Camera_cy", rclcpp::ParameterValue(double(0.0)));
    declare_parameter("Camera_k1", rclcpp::ParameterValue(double(0.0)));
    declare_parameter("Camera_k2", rclcpp::ParameterValue(double(0.0)));
    declare_parameter("Camera_p1", rclcpp::ParameterValue(double(0.0)));
    declare_parameter("Camera_p2", rclcpp::ParameterValue(double(0.0)));
    declare_parameter("Camera_p3", rclcpp::ParameterValue(double(0.0)));

}

//initilize
void stereo_camera_pub_node::init()
{

    get_parameter("video_path", video_path_);
    get_parameter("enable_camera", enable_camera_);
    get_parameter("camera_num", camera_num_);
    get_parameter("pub_from_image", pub_from_image_);
    get_parameter("image_path", image_directory_);
    get_parameter("frame_id", frame_id_);
    get_parameter("is_grayscale", is_grayscale_);

    get_parameter("Camera_fx", Camera_fx_);
    get_parameter("Camera_fy", Camera_fy_);
    get_parameter("Camera_cx", Camera_cx_);
    get_parameter("Camera_cy", Camera_cy_);
    get_parameter("Camera_k1", Camera_k1_);
    get_parameter("Camera_k2", Camera_k2_);
    get_parameter("Camera_p1", Camera_p1_);
    get_parameter("Camera_p2", Camera_p2_);
    get_parameter("Camera_p3", Camera_p3_);

    get_parameter("fps", fps_);

    RCLCPP_INFO(this->get_logger(), "%s", video_path_.data());
    RCLCPP_INFO(this->get_logger(), "%d, FPS %d", pub_from_image_, fps_);

    // Publish from image
    if(pub_from_image_){
        using std::filesystem::directory_iterator;
        using std::filesystem::directory_entry;
        for(directory_entry image_file_itr : directory_iterator(image_directory_)){
            image_paths_.push_back(image_file_itr.path());
        }
        
        std::sort(image_paths_.begin(), image_paths_.end());
        
        image_path_itr_ = image_paths_.begin();
        
        

        frame_count_ = image_paths_.size();

        cv::Mat image_info = cv::imread(image_paths_.front());
        height_ = image_info.rows;
        width_ = image_info.cols;
    }

    // Publish from video
    else{
        if(enable_camera_){
            video_.open(camera_num_);
        }

        else{
            video_.open(video_path_);
        }

        if(!video_.isOpened()){
            RCLCPP_ERROR(this->get_logger(), "Can't open video");
            //throw std::runtime_error("Can't open video");
        }

        height_ = video_.get(cv::CAP_PROP_FRAME_HEIGHT);
        width_ = video_.get(cv::CAP_PROP_FRAME_WIDTH);
        fps_ = video_.get(cv::CAP_PROP_FPS);
        frame_count_ = video_.get(cv::CAP_PROP_FRAME_COUNT);

    }

    image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    left_image_pub_ = image_transport_->advertise(node_name_ + "/" + frame_id_ + "/left", 10);
    right_image_pub_ = image_transport_->advertise(node_name_ + "/" + frame_id_ + "/right", 10);

    //rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    
    //left_image_pub1_ = this->create_publisher<sensor_msgs::msg::Image>(node_name_ + "/left/image_raw", rclcpp::QoS(10));
    //right_image_pub1_ = this->create_publisher<sensor_msgs::msg::Image>(node_name_ + "/right/image_raw", rclcpp::QoS(10));
    pubsize_ = cv::Size(uint(width_/2), height_);
    left_image_start_point = cv::Point(0,0);
    right_image_start_point = cv::Point(uint(width_/2), 0);

    left_image_roi = cv::Rect(left_image_start_point, pubsize_);
    right_image_roi = cv::Rect(right_image_start_point, pubsize_);

    int freq = 1000 / fps_;
    std::cout << freq << std::endl;

    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(node_name_ + "/camera_info", 10);

    if(pub_from_image_){
        timer_image_pub_ = this->create_wall_timer(std::chrono::milliseconds(freq),
                                    std::bind(&stereo_camera_pub_node::ImagePublishTimerCallback, this));
    }
    else{
        timer_image_pub_ = this->create_wall_timer(std::chrono::milliseconds(freq),
                                     std::bind(&stereo_camera_pub_node::VideoPublishTimerCallback, this));
    }

    timer_camera_info_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                                 std::bind(&stereo_camera_pub_node::TimerCallback2CameraInfo, this));

    
    camera_info.header.frame_id = "davinci";
    camera_info.height = height_;
    camera_info.width = width_;
    
    
    camera_info.k[0] = Camera_fx_;
    camera_info.k[4] = Camera_fy_;
    camera_info.k[2] = Camera_cx_;
    camera_info.k[5] = Camera_cy_;

    camera_info.d.resize(9);
    camera_info.d[0] = Camera_k1_;
    camera_info.d[1] = Camera_k2_;
    camera_info.d[2] = Camera_p1_;
    camera_info.d[3] = Camera_p2_;
    camera_info.d[4] = Camera_p3_;
    camera_info.p[3] = 100;
    
    RCLCPP_INFO(this->get_logger(), "Create instance");

}

// publish left image
void stereo_camera_pub_node::publish_left_camera(cv::Mat image)
{
    std_msgs::msg::Header header;
    header.stamp = current_frame_time_;
    header.frame_id = "sss";
    sensor_msgs::msg::Image::SharedPtr image_msg;
    if(this->is_grayscale_){
        cv::Mat gray_image;
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
        image_msg = cv_bridge::CvImage(header, "mono8", gray_image).toImageMsg();
    }
    else{
        image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
        
        //this->left_image_pub1_->publish(*image_msg);
    }
    this->left_image_pub_.publish(image_msg);
}

// publish right image
void stereo_camera_pub_node::publish_right_camera(cv::Mat image)
{
    std_msgs::msg::Header header;
    header.stamp = current_frame_time_;
    header.frame_id = "sss";
    sensor_msgs::msg::Image::SharedPtr image_msg;
    if(this->is_grayscale_){
        cv::Mat gray_image;
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
        image_msg = cv_bridge::CvImage(header, "mono8", gray_image).toImageMsg();
    }
    else{
        image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
        
        //this->left_image_pub1_->publish(*image_msg);
    }
    this->right_image_pub_.publish(image_msg);
    //this->right_image_pub1_->publish(*image_msg);
}

// timer callback that publish video frame from video
void stereo_camera_pub_node::VideoPublishTimerCallback()
{
    video_ >> frame_;

    if(frame_.empty()){
        RCLCPP_ERROR(this->get_logger(), "No Frame in video");
        rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "pub");
    cv::Mat left_image = frame_(left_image_roi);
    cv::Mat right_image = frame_(right_image_roi);

    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    current_frame_time_ = ros_clock.now();
    
    //this->camera_info_pub_->publish(camera_info);
    
    publish_left_camera(left_image);
    publish_right_camera(right_image);   
}

// timer callback that publish image frame from image
void stereo_camera_pub_node::ImagePublishTimerCallback()
{
    frame_ = cv::imread(*image_path_itr_);
    image_path_itr_++;
    
    cv::Mat left_image = frame_(left_image_roi);
    cv::Mat right_image = frame_(right_image_roi);

    current_frame_time_ = rclcpp::Time();

    RCLCPP_INFO(this->get_logger(), "%s", (*image_path_itr_).data());

    SetTimeStamp();

    publish_left_camera(left_image);
    publish_right_camera(right_image);

    if(image_paths_.end() == image_path_itr_){
        RCLCPP_ERROR(this->get_logger(), "No image to publish");
        rclcpp::shutdown();
    }
}

// timer callback that publish camera info
void stereo_camera_pub_node::TimerCallback2CameraInfo()
{
    camera_info_pub_->publish(camera_info);
}

// set time stamp for header of topic
void stereo_camera_pub_node::SetTimeStamp()
{
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    current_frame_time_ = ros_clock.now();
}