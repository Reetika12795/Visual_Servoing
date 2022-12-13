#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist


class path_planning:
    def __init__(self): 
    
        self.x_pos_25 = 0
        self.y_pos_25 = 0
        self.z_ori_25 = 0
        # self.x_pos_701 = 0
        # self.y_pos_701 = 0
        # self.z_ori_701 = 0

        print("Initalizing publisher and subscriber.")
        rospy.init_node("scan_move_node")
        rospy.Subscriber("/aruco_single/pose", PoseStamped, self.callback_25)
        # rospy.Subscriber("/aruco_single_2/pose", PoseStamped, self.callback_701)
        # vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # vel_msg = Twist()
        print('Subscribed and publishing.')
        rospy.sleep(5)


    def callback_25(self,data):
        self.x_pos_25 = data.pose.position.x
        self.y_pos_25 = data.pose.position.y
        self.z_ori_25 = data.pose.orientation.z


    # def callback_701(self,data):
    #     self.x_pos_701 = data.pose.position.x
    #     self.y_pos_701 = data.pose.position.y
    #     self.z_ori_701 = data.pose.orientation.z


    def printing(self):
        print("Position of ID-25 - \n\t x_pos : ",self.x_pos_25,"\n\t y_pos : ", 		      
        self.y_pos_25,"\n\t z_ori : ",self.z_ori_25)
        # print("Position of ID-701 - \n\t x_pos : ",self.x_pos_701,"\n\t y_pos : ", 
        # self.y_pos_701,"\n\t z_ori : ",self.z_ori_701)
        
        
# To publish in /cmd_vel, use -
#	vel_msg.linear.x = $value
#	vel_msg.linear.y = $value
#	vel_msg.angular.z = $value


if __name__ =="__main__":
    print("You are in Main now!")
    path = path_planning()
    while True:
        path.printing()
        rospy.sleep(1)
        rospy.spin()
