#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import csv
import numpy as np

file_1 = open('/home/masters/catkin_ws/src/my_aruco_tracker/src/data25.csv', 'a')
file_2 = open('/home/masters/catkin_ws/src/my_aruco_tracker/src/data701.csv', 'a')

w_1 = csv.writer(file_1)
w_2 = csv.writer(file_2)


def callback_1(data):
    w_1.writerow([data.header.seq, data.header.stamp, data.header.frame_id,
                data.pose.position.x, data.pose.position.y,data.pose.position.z, data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z, 
                data.pose.orientation.w])
def callback_2(data):
    w_2.writerow([data.header.seq, data.header.stamp, data.header.frame_id,
                data.pose.position.x, data.pose.position.y,data.pose.position.z, data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z, 
                data.pose.orientation.w])


def track_marker():
    rospy.init_node('tracking_data', anonymous=True)
    rospy.Subscriber('/aruco_single/pose', PoseStamped, callback_1)
    rospy.Subscriber('/aruco_single_2/pose', PoseStamped, callback_2)
    rospy.spin()


if __name__ == '__main__':
    print('Node to write tracked data started .......')
    w_1.writerow(['seq', 'time_stamp', 'frame_id', 'px', 'py','pz','qx','qy', 'qz', 'qw'])
    w_2.writerow(['seq', 'time_stamp', 'frame_id', 'px', 'py', 'pz','qx','qy','qz', 'qw'])
    track_marker()
    file_1.close()
    file_2.close()
    print('Done......')
