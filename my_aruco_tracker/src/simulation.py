
import math
import time
import matplotlib.pyplot as plt
#from get_aruco_coord import *

class Robot:
    def __init__(self,name,max_speed ,init_pos):
        self.__x = init_pos[0]
        self.__y = init_pos[1]
        self.__theta = init_pos[2]* math.pi/180
        self.__name = name

        self.__max_speed = max_speed


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
        if (forward_speed > self.__max_speed) :
            self.__forward_speed =self.__max_speed
        else:
            self.__forward_speed = forward_speed
    def get_forward_speed(self): 
        return self.__forward_speed
    def set_rotational_speed(self, rotational_speed):
        if(rotational_speed > self.__max_speed):
            self.__rotational_speed = self.__max_speed
        elif(rotational_speed <  - self.__max_speed):
            self.__rotational_speed =  - self.__max_speed
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




class Controller:
    def __init__(self, forward_speed_gain, rotational_speed_gain):
        self.forward_speed_gain = forward_speed_gain
        self.rotational_speed_gain = rotational_speed_gain

    def robot_orientation(self, current_position, target_position): #returns the forward and rotational speed
        difference = [(target_position[0] - current_position[0]), (target_position[1] - current_position[1])]
        theta_change = math.atan2(  target_position[1] - current_position[1],target_position[0] - current_position[0]) - current_position[2] 
        self.rotationalspeed = self.rotational_speed_gain * float(theta_change)

        print("theta_change:", theta_change*(180/math.pi),"\n")
        """if( abs(theta_change)*(180/math.pi) > 30):
            self.forwardspeed = 0
            print("correct theta")
        else:"""
        a = self.forward_speed_gain * difference
        self.forwardspeed = math.sqrt((a[0])**2 + (a[1])**2) #resultant of x and y linear components speed
        return (self.forwardspeed , self.rotationalspeed)

initial = [0,0,-90]

import time 
robot = Robot("Jimmy", 0.7, initial)
controller = Controller(9, 6)
iteration_time_sec = 0.1
robot.set_dtime(iteration_time_sec)
target_pos = [2,2]
t_k = time.time()
final_time = time.time();
k=1
plt.plot([initial[0]-0.1,initial[0]+0.1,initial[0]-0.1,initial[0]+0.1], [initial[1]-0.1,initial[1]-0.1,initial[1]+0.1,initial[1]+0.1])# shows the position of the robot (X,Y) at each iteration

while True:
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
        print("target pose", target_pos)
        # rostopic pub mobile_base_controller/cmd_vel geometry_msgs/Twist -r 3 -- '[0.5,0.0,0.0]' '[0.0, 0.0, 0.0]'

        
        print('Distance', distance)
        plt.scatter(robot.get_position()[0], robot.get_position()[1],c='red')# shows the position of the robot (X,Y) at each iteration
        final_time = time.time()
        k = k+1
        
print("{} arrived to the target!".format(robot.get_name()))
plt.show()