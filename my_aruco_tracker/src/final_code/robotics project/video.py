import cv2
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from houghDetection import *

bridge = CvBridge()
def callback(img):
    cap = bridge.imgmsg_to_cv2(img, "bgr8")
    print(cap)
    cv2.imshow("output",cap)
    cv2.waitKey(1)
    ordre = main(cap)


# cap = cv2.VideoCapture(0)
# print("press q to exit")
rospy.init_node("cam_cap_node")
rospy.Subscriber("/camera/image",Image, callback)
rospy.spin()
	
	
###---------------------------------------------------------------###

###---------------------------------------------------------------###




#When everything done, release the capture

