#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class tb3_lidar:

    def __init__(self):

        self.fwd_20 = 0
        self.fwd_340 = 0
        self.fwl_50 = 0
        self.lft_90 = 0
        self.rgt_270 = 0
        self.fwr_315 = 0
        # self.fwd_speed = 0.08
        # self.ang_speed = 0.06
        rospy.init_node('scan_move_node')
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        print("initiation!")
        # self.rate = rospy.Rate(10)
        # rospy.on_shutdown(self.bye)
        # rospy.spin()
        rospy.sleep(0.20)


    def callback(self, msg):
        #print(len(msg.ranges))
        self.fwd_20 = msg.ranges[20]
        self.fwl_50 = msg.ranges[50]
        self.lft_90 = msg.ranges[90]
        self.rgt_270 = msg.ranges[270]
        self.fwr_315 = msg.ranges[315]
        self.fwd_340 = msg.ranges[340]
        # self.printing()

    def data_return(self):
        self.printing()
        return [self.lft_90, self.fwl_50, self.fwd_20, self.fwd_340, self.fwr_315, self.rgt_270]

    def printing(self):
        # while not rospy.is_shutdown():
            print(f'{self.fwd_20 = }')
            print(f'{self.lft_90 = }')
            print(f'{self.rgt_270 = }')
            print("------------------")
        # rospy.spin()

p = tb3_lidar()
p.printing()
# tb3_lidar()