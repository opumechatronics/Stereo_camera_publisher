#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

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

//const std::string video_path = "aa";

int
main(int argc, char **argv)
{
    ros::init (argc, argv, "img_publisher");
    ros::NodeHandle nh("~"); 
    image_transport::ImageTransport it(nh);
    image_transport::Publisher left_image_pub_ = it.advertise("/camera/left/image", 10);
    image_transport::Publisher right_image_pub_ = it.advertise("/camera/right/image", 10);
    cv::Mat image;
    cv::VideoCapture video("/home/bkmel/catkin_ws/test.mp4");

    int width = video.get(CV_CAP_PROP_FRAME_WIDTH);
    int height = video.get(CV_CAP_PROP_FRAME_HEIGHT);
    int frame_count = video.get(CV_CAP_PROP_FRAME_COUNT);
    int fps = video.get(CV_CAP_PROP_FPS);

    cv::Size resize(int(width / 2), height);

    cv::Rect lroi(cv::Point(0,0), resize);
    cv::Rect rroi(cv::Point(int(width / 2), 0), resize);

    video >> image;
    cv::Mat left_image = image(lroi);
    cv::Mat right_iamge = image(rroi);

    cv::imshow("test", left_image);
    cv::waitKey(0);

    if (!video.isOpened())
    {
        ROS_INFO("failed to open camera.");
        return -1;
    }
    ros::Rate looprate (30); // capture image at 10Hz
    while(ros::ok()) {
        video >> image;


    }
    return 0;
}