#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import rosparam
import message_filters
from std_msgs.msg import String

import time
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def main():
    video_file_path = "/home/bkmn/thesis/20200218_102324_0_0.avi"

    #Set up Node 
    rospy.init_node("video_publsiher", anonymous=True)
    left_image_pub  = rospy.Publisher("/camera/davincixi/left_image", Image, queue_size=10)
    right_image_pub = rospy.Publisher("/camera/davincixi/right_image", Image, queue_size=10)
    #right_image_pub = rospy.Publisher("/camera/right_image", Image, queue_size=10)
    left_camera_info = rospy.Publisher("image_left/camera_info", CameraInfo, queue_size=10)

    video = cv2.VideoCapture(video_file_path)

    if not video.isOpened():
        print("not exist")
        return

    frame_count = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = int(video.get(cv2.CAP_PROP_FPS))
    rate = rospy.Rate(fps)
    width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    left_width = int(width//2)
    half_width = left_width
    half_height = height//2

    #camera info
    camera_info = CameraInfo()
    camera_info.width = left_width
    camera_info.height = height
    
    #camera info publisher
    left_camera_info.publish(camera_info)

    pre_time = -1
    start = time.time()

    #for i in range(frame_count):
    while not rospy.is_shutdown():
    #while video.grab() and not rospy.is_shutdown():
        now_time = time.time()
        if pre_time == -1:
            pre_time = now_time
        
        progress_time = now_time - pre_time

        # sec to msec
        progress_time = progress_time * 1000
        progress_frame = progress_time / (1000 / 30.0)
        print progress_time, progress_frame

        if progress_frame > frame_count:
            break

        #tmp, img = video.read()
        video.set(cv2.CAP_PROP_POS_FRAMES, int(progress_frame))
        tmp, img = video.read()
        if not tmp:
            break
        
        cv2.imwrite("original.jpg", img)
        
        #splite image and resize to half size
        img_left = img[0:height, 0:left_width]
        img_right = img[0:height, left_width:width]
        #img_left = cv2.resize(img_left, (half_height, half_width))
        #img_right = cv2.resize(img_right, (half_height, half_width))
        
        ros_now_time = rospy.Time.now()   
        #left image publisher
        try:
            img_msg_left = bridge.cv2_to_imgmsg(img_left, "bgr8")
            img_msg_left.header.stamp = ros_now_time
            img_msg_left.header.frame_id = "camera_link"
            left_image_pub.publish(img_msg_left)

        except CvBridgeError as err:
            print(err)
        
        #right image publisher
        try:
            img_msg_right = bridge.cv2_to_imgmsg(img_right, "bgr8")
            img_msg_right.header.stamp = ros_now_time
            img_msg_right.header.frame_id = "camera_link"
            right_image_pub.publish(img_msg_right)

        except CvBridgeError as err:
            print(err)
        print(now_time)
        #rate.sleep()
    print time.time() - start

        
        

if __name__ == '__main__':
    main()