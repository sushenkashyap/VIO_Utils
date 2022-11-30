#!/usr/bin/env python3
from __future__ import print_function
import rospy
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point
from sensor_msgs.msg import Joy

import sys
import json
from math import sqrt
from collections import deque

import time

gt_pos = Point()
dist = Float32()
gt_pos.x = 0
gt_pos.y = 0
dist = 0
n = 0
err_sq = 0

def calc_error(data):
    global gt_pos, n, err_sq
    est_pos = Point()
    rms_err = Float32()
    est_pos = data.pose.pose.position
    
    n += 1
    if n<100:    
        err_sq += (gt_pos.x - est_pos.x)**2 + (gt_pos.y - est_pos.y)**2
        rms_err = sqrt(err_sq/n)
        err_pub.publish(rms_err)
    else:
        n = 0;
        err_sq = 0;

def update_gt(msg):
    global gt_pos, dist
    dist += sqrt((msg.pose.pose.position.x - gt_pos.x)**2 + (msg.pose.pose.position.y - gt_pos.y)**2)
    gt_pos = msg.pose.pose.position
    dist_pub.publish(dist)

if __name__ == '__main__':
    est_odom_topic = sys.argv[1]
    ground_truth_topic = sys.argv[2] 


    #Node and msg initialization
    rospy.init_node('calc_error_node', anonymous=True)

    err_pub = rospy.Publisher('/rms_err', Float32, queue_size=1)
    dist_pub = rospy.Publisher('/dist', Float32, queue_size=1)

    #Subscription to the topic
    gt_data = rospy.Subscriber(ground_truth_topic, Odometry, update_gt)
    msg = rospy.Subscriber(est_odom_topic, Odometry, calc_error)

    try:
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
            pass
