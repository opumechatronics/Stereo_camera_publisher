#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import rosparam
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Vector3
import tf

import numpy as np
import rospy

from cv_bridge import CvBridge, CvBridgeError

class orb_slam_printer():
    times = -1
    def __init__(self):
        self.sub = rospy.Subscriber("/orb_slam2_stereo/pose", PoseStamped, self.posestamp_callback,queue_size=10)


    def posestamp_callback(self, posestamp):
        
        t = posestamp.header.stamp
        
        if self.times == -1:
            self.times = t - t
        else:
            self.times = t - self.times
        

        x = posestamp.pose.position.x
        y = posestamp.pose.position.y
        z = posestamp.pose.position.z

        e = tf.transformations.euler_from_quaternion((posestamp.pose.orientation.x, 
                                                        posestamp.pose.orientation.y, 
                                                        posestamp.pose.orientation.z, 
                                                        posestamp.pose.orientation.w))

        print self.times

        print x, y, z, e[0], e[1], e[2]
    
        
def main():
    rospy.init_node("slam_pose_printer")
    node = orb_slam_printer()
    rospy.spin()

if __name__ == '__main__':
    main()