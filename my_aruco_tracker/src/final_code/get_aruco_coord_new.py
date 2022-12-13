#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped


class path_planning:
    def __init__(self): 
        self.x_pos_25 = 0
        self.y_pos_25 = 0
        self.z_pos_25 = 0
        self.x_ori_25 = 0
        self.y_ori_25 = 0
        self.z_ori_25 = 0

        # self.x_pos_701 = 0
        # self.y_pos_701 = 0
        # self.z_ori_701 = 0
        # print("Initalizing publisher and subscriber.")
        rospy.init_node("scan_move_node")
        rospy.Subscriber("/aruco_single/pose", PoseStamped, self.callback_25)
        # rospy.Subscriber("/aruco_single_2/pose", PoseStamped, self.callback_701)
        # print('Subscribed and publishing.')
        rospy.sleep(0.1)
        # print("i'm the bug!")


    def callback_25(self,data):
        self.x_pos_25 = data.pose.position.x
        self.y_pos_25 = data.pose.position.y
        self.z_pos_25 = data.pose.position.z
        self.x_ori_25 = data.pose.orientation.x
        self.y_ori_25 = data.pose.orientation.y
        self.z_ori_25 = data.pose.orientation.z


    def data_return(self):
        self.rot= [self.x_ori_25,self.y_ori_25,self.z_ori_25]
        self.transl= [[self.x_pos_25],[self.y_pos_25],[self.z_pos_25]]
        return self.rot, self.transl


    # def callback_701(self,data):
    #     self.x_pos_701 = data.pose.position.x
    #     self.y_pos_701 = data.pose.position.y
    #     self.z_ori_701 = data.pose.orientation.z


    def printing(self):
        # initial = []
        # initial = [self.x_pos_25,self.y_pos_25,self.z_ori_25]
        # target = []
        # target = [self.x_pos_701,self.y_pos_701]
        print("Position of ID-25 - \n\t x_pos : ",self.x_pos_25,"\n\t y_pos : ", 		      
        self.y_pos_25,"\n\t z_ori : ",self.z_ori_25)
        # print("Position of ID-701 - \n\t x_pos : ",self.x_pos_701,"\n\t y_pos : ", 
        # self.y_pos_701,"\n\t z_ori : ",self.z_ori_701)
        # return initial,target


pp = path_planning()
pp.printing()