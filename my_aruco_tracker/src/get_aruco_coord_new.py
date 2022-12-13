#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

class path_planning:
    def __init__(self): 

        self.fwd_10 = 0
        self.fwl_50 = 0
        self.fwr_315 = 0

        self.x_pos_25 = 0
        self.y_pos_25 = 0
        self.z_pos_25 = 0
        self.x_ori_25 = 0
        self.y_ori_25 = 0
        self.z_ori_25 = 0
        self.w_ori_25 = 0

        self.x_pos_701 = 0
        self.y_pos_701 = 0
        self.z_pos_701 = 0
        self.x_ori_701 = 0
        self.y_ori_701 = 0
        self.z_ori_701 = 0
        self.w_ori_701 = 0
        # print("Initalizing publisher and subscriber.")
        rospy.init_node("scan_move_node")
        rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        rospy.Subscriber("/aruco_single/pose", PoseStamped, self.callback_25)
        rospy.Subscriber("/aruco_single_2/pose", PoseStamped, self.callback_701)
        # print('Subscribed and publishing.')
        rospy.sleep(0.40)
        # print("i'm the bug!")

    def callback_lidar(self, msg):
        #print(len(msg.ranges))
        self.fwd_10 = msg.ranges[0]
        self.fwl_50 = msg.ranges[50]
        self.fwr_315 = msg.ranges[315]


    def callback_25(self,data):
        self.x_pos_25 = data.pose.position.x
        self.y_pos_25 = data.pose.position.y
        self.z_pos_25 = data.pose.position.z
        self.x_ori_25 = data.pose.orientation.x
        self.y_ori_25 = data.pose.orientation.y
        self.z_ori_25 = data.pose.orientation.z
        self.w_ori_25 = data.pose.orientation.w

    def data_return_lidar(self):
        # self.printing()
        return self.fwl_50, self.fwd_10, self.fwr_315

        
    def data_return(self):
        self.rot_rob= [self.x_ori_25, self.y_ori_25, self.z_ori_25, self.w_ori_25]
        self.transl_rob= [[self.x_pos_25],[self.y_pos_25],[self.z_pos_25]]
        self.rot_target= [self.x_ori_701,self.y_ori_701,self.z_ori_701, self.w_ori_701]
        self.transl_target= [[self.x_pos_701],[self.y_pos_701],[self.z_pos_701]]
        return self.rot_rob, self.transl_rob, self.rot_target, self.transl_target


    def callback_701(self,data):
        self.x_pos_701 = data.pose.position.x
        self.y_pos_701 = data.pose.position.y
        self.z_pos_701 = data.pose.position.z
        self.x_ori_701 = data.pose.orientation.x
        self.y_ori_701 = data.pose.orientation.y
        self.z_ori_701 = data.pose.orientation.z
        self.w_ori_701 = data.pose.orientation.w



    def printing(self):
        # initial = []
        # initial = [self.x_pos_25,self.y_pos_25,self.z_ori_25]
        # target = []
        # target = [self.x_pos_701,self.y_pos_701]
        print("Position of ID-25 - \n\t x_pos : ",self.x_pos_25,"\n\t y_pos : ", 		      
        self.y_pos_25,"\n\t z_ori : ",self.z_ori_25)
        # print("i'm the bug!")
        # print("Position of ID-701 - \n\t x_pos : ",self.x_pos_701,"\n\t y_pos : ", 
        # self.y_pos_701,"\n\t z_ori : ",self.z_ori_701)
        # # return initial,target
        print("------------------")
        print(f'{self.fwl_50 = }')
        print(f'{self.fwd_10 = }')
        print(f'{self.fwr_315}')
        print("------------------")


pp = path_planning()
pp.printing()