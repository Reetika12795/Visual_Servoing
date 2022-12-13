#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped,Twist
import numpy as np


def callback_1(data):
	print("Position of ID-25 - \n\t x_pos : ",data.pose.position.x,"\n\t y_pos : ", 		      
     data.pose.position.y,"\n\t z_ori : ",data.pose.orientation.z)
    # return data.pose.position.x,data.pose.position.y,data.pose.orientation.z

def callback_2(data):
	print("Position of ID-701 - \n\t x_pos : ",data.pose.position.x,"\n\t y_pos : ", 		       
    data.pose.position.y,"\n\t z_ori : ",data.pose.orientation.z)
# #	print(type(data.pose.position.x))
    
# def get_pose_robot(x,y,z):
# a,b,c = callback_1(data)
# print("data.pose.position.x",a)
# print(b)
# print(c)

    



# def track_marker():
rospy.init_node('tracking_data', anonymous=True)
rospy.Subscriber('/aruco_single/pose', PoseStamped, callback_1)
rospy.Subscriber('/aruco_single_2/pose', PoseStamped, callback_2)

cmd_pub = rospy.Publisher('/cmd_vel', Twist)
cmd_pub.publish()
rospy.spin()


# if __name__ == '__main__':
#     print('Node to write tracked data started .......')
#     track_marker()
#     print('Done......')
    
    
#data from geometry_msgs.msg --> PoseStamped
  
#([data.header.seq, data.header.stamp, data.header.frame_id,
#  data.pose.position.x, data.pose.position.y, data.pose.position.z,
#  data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
