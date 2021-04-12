#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <iostream>
#include <fstream>
#include <cstdio>

ros::Time start;
std::ofstream csv;

void tf_quat_to_rpy(double& roll, double& pitch, double &yaw, tf::Quaternion quat)
{
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

}

void posestampCallback(const geometry_msgs::PoseStamped& posestamped)
{
    static bool init = true;
    
    geometry_msgs::Quaternion geo_q = posestamped.pose.orientation;
    ros::Time time = posestamped.header.stamp;
    double yaw, roll, pitch;
    tf::Quaternion tf_q;
    quaternionMsgToTF(geo_q , tf_q);
    tf_quat_to_rpy(roll, pitch, yaw, tf_q);

    if(init){
        init = false;
        start = time;
    }

    ros::Duration ros_duration = time - start;
    char str[256];
    sprintf(str, "%u.%u %f %f %f %f %f %f", ros_duration.sec, ros_duration.nsec, posestamped.pose.position.x, posestamped.pose.position.y, posestamped.pose.position.z,
                                    roll, pitch, yaw);
    csv << str << std::endl;

    //ROS_INFO("ROS: %s", str);
    //csv << str << std::endl;
    //ROS_INFO("ROS: %u.09%u", ros_duration.sec, ros_duration.nsec);
    //ROS_INFO("x:%d, y:%d, z:%d", posestamped.pose.position.x, posestamped.pose.position.y, posestamped.pose.position.z);
    //ROS_INFO("roll:%d, pitch:%d, yaw:%d", roll, pitch, yaw);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam_pose_printer");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/orb_slam2_stereo/pose", 10, posestampCallback);

    csv.open("/home/bkmn/catkin_ws/pose_stamp_output.csv");
    csv << "time(sec) x y z roll pitch yaw" << std::endl;
    ros::spin();

    return 0;
}