#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import rosparam
import message_filters
from std_msgs.msg import String

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
    left_image_pub = rospy.Publisher("/camera/davincixi/left_iamge", Image, queue_size=10)
    right_image_pub = rospy.Publisher("/camera/davincixi/right_image", Image, queue_size=10)

    video = cv2.VideoCapture(video_file_path)

    if not video.isOpened():
        print("not exist")
        return

    print("exist")

    fps = video.get(cv2.CAP_PROP_FPS)
    rate = rospy.Rate(fps)
    width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    left_width = int(width//2)
    half_width = left_width
    half_height = height//2
    
    ret, img = video.read()

    while True:
    #while video.grab() and not rospy.is_shutdown():
        
        """
        tmp, img = video.retrieve()
        if not tmp:
            break
        
        cv2.imwrite("original.jpg", img)
        """
        #splite image and resize to half size
        img_left = img[0:height, 0:left_width]
        img_right = img[0:height, left_width:width]
        img_left = cv2.resize(img_left, (half_height, half_width))
        img_right = cv2.resize(img_right, (half_height, half_width))
        
        now_time = rospy.Time.now()   
        #left image publisher
        try:
            img_msg_left = bridge.cv2_to_imgmsg(img_left, "bgr8")
            img_msg_left.header.stamp = now_time
            img_msg_left.header.frame_id = "camera_link"
            left_image_pub.publish(img_msg_left)

        except CvBridgeError as err:
            print(err)
        
        #right image publisher
        try:
            img_msg_right = bridge.cv2_to_imgmsg(img_right, "bgr8")
            img_msg_right.header.stamp = now_time
            img_msg_right.header.frame_id = "camera_link"
            right_image_pub.publish(img_msg_right)

        except CvBridgeError as err:
            print(err)
        
        rate.sleep()
        

        
        

if __name__ == '__main__':
    main()