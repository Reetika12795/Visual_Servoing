#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class tb3_lidar:

    def __init__(self):

        self.dist_front = 0.17
        self.dist_rt_lft = 0.15
        self.fwd = 0
        self.fwl = 0
        self.lft = 0
        self.rgt = 0
        self.fwr = 0
        self.fwd_speed = 0.02
        self.ang_speed = 0.2
        rospy.init_node('scan_move_node')
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback)

        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.bye)


    def callback(self, msg):
        #print(len(msg.ranges))
        self.fwd = msg.ranges[0]
        self.fwl = msg.ranges[45]
        self.lft = msg.ranges[90]
        self.rgt = msg.ranges[270]
        self.fwr = msg.ranges[315]

    def move_robot(self):
        print("Robot started moving!")
        while not rospy.is_shutdown():

            if self.fwd > self.dist_front:
                self.vel_msg.linear.x = self.fwd_speed
                self.vel_msg.angular.z = 0

            if self.fwd < self.dist_front and self.fwr < self.dist_rt_lft:
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = self.ang_speed

            if self.fwd < self.dist_front and self.fwl < self.dist_rt_lft:
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = -self.ang_speed

            if self.lft < self.dist_rt_lft:
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = -self.ang_speed

            if self.rgt < self.dist_rt_lft:
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = self.ang_speed

            if self.fwl < self.dist_rt_lft:
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = -self.ang_speed

            if self.fwr < self.dist_rt_lft:
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = self.ang_speed

            # if self.fwd > self.dist_front and self.lft < self.dist_rt_lft and self.rgt < self.dist_rt_lft and self.fwl < self.dist_rt_lft and self.fwr < self.dist_rt_lft:
            #     self.vel_msg.linear.x = -self.fwd_speed
            #     self.vel_msg.angular.z = 0                
            
            # self.printing()
            self.vel_pub.publish(self.vel_msg)
            self.printing()
        
        rospy.spin()
        

    def printing(self):
            print(f'{self.fwd = }')
            print(f'{self.lft = }')
            print(f'{self.rgt = }')
            print("------------------")

    def bye(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.vel_pub.publish(self.vel_msg)
        rospy.loginfo("Robot stopped!")        
        


if __name__ == '__main__':
    go_bot = tb3_lidar()
    try:
        go_bot.move_robot()
    except rospy.ROSInterruptException:
        pass

