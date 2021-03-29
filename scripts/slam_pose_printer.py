#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import rosparam
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

import numpy as np
import rospy

from cv_bridge import CvBridge, CvBridgeError

class orb_slam_printer():
    def __init__(self):
        self.sub = rospy.Subscriber("/orb_slam2_mono/pose", PoseStamped, self.posestamp_callback,queue_size=1)


    def posestamp_callback(self, posestamp):
        x = posestamp.pose.position.x
        y = posestamp.pose.position.y
        z = posestamp.pose.position.z

        print x, y, z
        
def main():
    rospy.init_node("slam_pose_printer")
    node = orb_slam_printer()
    rospy.spin()

if __name__ == '__main__':
    main()