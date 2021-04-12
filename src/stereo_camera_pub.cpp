#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>

class StereoCameraPub
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher left_image_pub_;
    image_transport::Publisher right_image_pub_;
    cv::VideoCapture video;

public:
    StereoCameraPub() : it_(nh_)
    {
    }
    ~StereoCameraPub();
};

const std::string video_path("/home/bkmn/thesis/20200218_102324_0_0.avi");

int main(int argc, char **argv)
{
    ros::init (argc, argv, "stereo_camera_publisher");
    ros::NodeHandle nh; 
    std::chrono::system_clock::time_point start, end;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher left_image_pub_ = it.advertise("/camera/davincixi/left_image", 10);
    image_transport::Publisher right_image_pub_ = it.advertise("/camera/davincixi/right_image", 10);
    ros::Publisher camera_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>("image_left/camera_info", 1000);
    ros::Time timestamp;

    cv::Mat image;
    cv::VideoCapture video(video_path);

    if (!video.isOpened())
    {
        ROS_INFO("failed to open camera.");
        return -1;
    }
    
    //cv::VideoCapture video("/home/bkmel/catkin_ws/test.mp4");

    int width = video.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = video.get(cv::CAP_PROP_FRAME_HEIGHT);
    int frame_count = video.get(cv::CAP_PROP_FRAME_COUNT);
    int fps = video.get(cv::CAP_PROP_FPS);

    cv::Size resize(int(width / 2), height);
    cv::Point left_image_start_point(0,0);
    cv::Point right_image_start_point(uint(width/2), 0);

    cv::Rect left_image_roi(left_image_start_point, resize);
    cv::Rect right_image_roi(right_image_start_point, resize);

    sensor_msgs::CameraInfoPtr linfo(new sensor_msgs::CameraInfo);
    linfo->height = height;
    linfo->width = width;
    linfo->header.frame_id = "camera_link";

    start = std::chrono::system_clock::now();
    
    ros::Rate looprate (30); // capture image at 10Hz
    int frames = 0;
    while(ros::ok()) {
        video >> image;
        ros::Time now_time = ros::Time::now();
        std::cout << now_time << std::endl;

        if (image.empty()){
            break;
        }

        frames++;

        cv::Mat left_image = image(left_image_roi);
        cv::Mat right_image = image(right_image_roi);
        sensor_msgs::ImagePtr left_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_image).toImageMsg();
        sensor_msgs::ImagePtr right_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", right_image).toImageMsg();

        left_image_msg->header.stamp = now_time;
        right_image_msg->header.stamp = now_time;
        linfo->header.stamp = now_time;

        camera_info_pub_.publish(linfo);

        left_image_pub_.publish(left_image_msg);
        right_image_pub_.publish(right_image_msg);

        looprate.sleep();

    }
    end = std::chrono::system_clock::now();
    const double time = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count());
    ROS_INFO("time %lf[msec]\n fps to time %lf[msec]", time, double(frame_count * 1000) / fps);
    ROS_INFO("Frame count video:%d count%d ", frame_count, frames);
    return 0;
}