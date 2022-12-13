#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
import math


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
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
        self.vel_msg = Twist()
        # rospy.Subscriber("/aruco_single_2/pose", PoseStamped, self.callback_701)
        # print('Subscribed and publishing.')
        rospy.sleep(5)
        
    def get_rotation (self, msg):
        global roll, pitch, yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        print (yaw)


    def callback_25(self,data):
        self.x_pos_25 = data.pose.position.x
        self.y_pos_25 = data.pose.position.y
        self.z_pos_25 = data.pose.position.z
        self.x_ori_25 = data.pose.orientation.x
        self.y_ori_25 = data.pose.orientation.y
        self.z_ori_25 = data.pose.orientation.z


    def data_return(self):
        rot= [self.x_ori_25,self.y_ori_25,self.z_ori_25]
        transl= [[self.x_pos_25],[self.y_pos_25],[self.z_pos_25]]
        return rot, transl


    # def callback_701(self,data):
    #     self.x_pos_701 = data.pose.position.x
    #     self.y_pos_701 = data.pose.position.y
    #     self.z_ori_701 = data.pose.orientation.z


    def printing(self):
        target = 90
        kp = 0.5
        # initial = []
        # initial = [self.x_pos_25,self.y_pos_25,self.z_ori_25]
        # target = []
        # target = [self.x_pos_701,self.y_pos_701]
        print("Position of ID-25 - \n\t x_pos : ",self.x_pos_25,"\n\t y_pos : ", 		      
        self.y_pos_25,"\n\t z_ori : ",self.z_ori_25)

        target_rad = target*math.pi/180
        self.vel_msg.angular.z = kp * (target_rad-0.0)

        # self.vel_msg.angular.z = 0.2
        # self.vel_pub.publish(self.vel_msg)
        # rospy.sleep(2)
        # self.vel_msg.angular.z = 0.0
        self.vel_pub.publish(self.vel_msg)
        # print("Position of ID-701 - \n\t x_pos : ",self.x_pos_701,"\n\t y_pos : ", 
        # self.y_pos_701,"\n\t z_ori : ",self.z_ori_701)
        # return initial,target

pp = path_planning()
pp.printing()


