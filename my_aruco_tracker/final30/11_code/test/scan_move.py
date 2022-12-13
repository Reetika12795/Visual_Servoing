#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def callback(msg):
    #print(len(msg.ranges))
    dist = 0.4
    lft_rt_angle = 0.14
    lft_rt = 0.13
    fwd = msg.ranges[0]
    fwl = msg.ranges[45]
    lft = msg.ranges[90]
    bck = msg.ranges[180]
    fwr = msg.ranges[315]
    rgt = msg.ranges[270]

    if fwd > dist and fwd != 0.0:
        vel_msg.linear.x = 0.04
        vel_msg.angular.z = 0.0

    if fwl < lft_rt_angle and fwl != 0.0:
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = -0.2

    if fwr < lft_rt_angle and fwr != 0.0:
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.2

    if fwd < dist and fwd != 0.0:
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0.2

    if lft < lft_rt and lft != 0.0:
        vel_msg.linear.x = 0
        vel_msg.angular.z = -0.2

    if rgt < lft_rt and rgt != 0.0:
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0.2

    vel_pub.publish(vel_msg)


    print(f'{lft = }')
    print(f'{fwl = }')
    print(f'{fwd = }')
    print(f'{fwr = }')
    print(f'{rgt = }')
    print("------------------")


rospy.init_node('scan_move_node')
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
vel_msg = Twist()
scan_sub = rospy.Subscriber('/scan', LaserScan, callback)

rate = rospy.Rate(10)

rospy.spin()

