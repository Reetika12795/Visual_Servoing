#!/usr/bin/env python3
# coding: utf-8

# In[67]:


import math
import time
import matplotlib.pyplot as plt
from get_aruco_coord import *
import rospy
from geometry_msgs.msg import PoseStamped,Twist


# In[68]:


class Robot:
    def __init__(self,name,max_speed ,init_pos):
        self.__x = init_pos[0]
        self.__y = init_pos[1]
        self.__theta = init_pos[2]* math.pi/180
        self.__name = name
        if max_speed < 0.1:#check for conditions
            self.__max_speed = max_speed
        else:
            self.__max_speed = 0.1
        self.__forward_speed = 0.05
        self.__rotational_speed = 0.0
        self.__Dtime = 0.0

    def set_name(self, name):
        self.__name = name
    def get_name(self):
        return self.__name
    def get_position(self):
        return [self.__x, self.__y, self.__theta]
    def set_dtime(self, dtime):
        self.__Dtime = dtime
    def set_forward_speed(self, forward_speed):
        if (forward_speed > 0.3) :
            self.__forward_speed = 0.3
        else:
            self.__forward_speed = forward_speed
    def get_forward_speed(self): 
        return self.__forward_speed
    def set_rotational_speed(self, rotational_speed):
        if(rotational_speed > 0.3):
            self.__rotational_speed = 0.3
        else:
            self.__rotational_speed = rotational_speed
    def get_rotational_speed(self):
        return self.__rotational_speed

    def update_position(self): # will update the position of the robot with the help of forward and rotational speed
        self.__theta = self.__theta + self.get_rotational_speed() * self.__Dtime
        velocity_x = self.get_forward_speed() * math.cos(self.__theta)
        velocity_y = self.get_forward_speed() * math.sin(self.__theta)
        self.__x = self.__x + velocity_x * self.__Dtime
        self.__y = self.__y + velocity_y * self.__Dtime
        


# In[69]:


class Controller:
    def __init__(self, forward_speed_gain, rotational_speed_gain):
        self.forward_speed_gain = forward_speed_gain
        self.rotational_speed_gain = rotational_speed_gain

    def robot_orientation(self, current_position, target_position): #returns the forward and rotational speed
        difference = [(target_position[0] - current_position[0]), (target_position[1] - current_position[1])]
        theta_change = math.atan2(  target_position[1] - current_position[1],target_position[0] - current_position[0]) - current_position[2] 
        self.rotationalspeed = self.rotational_speed_gain * float(theta_change)

        print("theta_change:", theta_change*(180/math.pi),"\n")
        if( abs(theta_change)*(180/math.pi) > 5):
            self.forwardspeed = 0
        else:
            a = self.forward_speed_gain * difference
            self.forwardspeed = math.sqrt((a[0])**2 + (a[1])**2) #resultant of x and y linear components speed
        return (self.forwardspeed , self.rotationalspeed)


# In[70]:


# def distance_to_target(current_position, target_position):
#     distance = math.sqrt((target_position[0]-current_position[0])**2 + (target_position[1]-current_position[1])**2 )
#     return distance


# In[71]:
rospy.init_node('scan_move_node')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
rate=rospy.Rate(50)


path = path_planning()
initial,target = path.printing()
#initial = [-0.10636874288320541,0.4602086544036865,-0.0060170689597272555] 
#target = [0.5653693079948425, 0.9323462843894958]
import time 
robot = Robot("Jimmy", 0.7, initial)
controller = Controller(1, 6)
iteration_time_sec = 0.001
robot.set_dtime(iteration_time_sec)
target_pos = target
# Initialize robot time:
t_k = time.time()
#robot.init_task(t_k)
final_time = time.time()
k=1
while True:
    #initial,target = path.printing()
    target_pos = target
    pos = robot.get_position()[:2]
    distance = math.sqrt((target_pos[0]-pos[0])**2 + (target_pos[1]-pos[1])**2 )
   

    if distance < 0.01:
        print("time taken to reach target is", final_time - t_k)
        break
    else:
        f, r = controller.robot_orientation(robot.get_position(), target_pos)
        # Compute forward and rotation speed with controller
        robot.set_forward_speed(f)
        robot.set_rotational_speed(r) # set speed to robot        
        # t_k = current_time        
        time.sleep(iteration_time_sec)        
        currrent_time = time.time()
        
        # update robot pos with t_k and current_time
        robot.update_position()
        print(f"step :{k} \n forwardspeed = ", robot.get_forward_speed(),"r_ speed : ", robot.get_rotational_speed(), " current position :" , robot.get_position())
        # rostopic pub mobile_base_controller/cmd_vel geometry_msgs/Twist -r 3 -- '[0.5,0.0,0.0]' '[0.0, 0.0, 0.0]'
        forwards = robot.get_forward_speed()
        rotations = robot.get_rotational_speed()
        twist = Twist()
        twist.linear.x = forwards*math.cos(robot.get_position()[2]) 
        # twist.linear.x=0.1
        twist.linear.y = forwards*math.sin(robot.get_position()[2]) 
        # twist.linear.y=0
        twist.linear.z =0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = rotations
        # twist.angular.z=0
        pub.publish(twist)
        # print('Distance', distance_to_target(robot.get_position()[:2], target_pos))
        # plt.plot(robot.get_position()[0], robot.get_position()[1],'g^')# shows the position of the robot (X,Y) at each iteration
        final_time = time.time()
        k = k+1
        
print("{} arrived to the target!".format(robot.get_name()))

