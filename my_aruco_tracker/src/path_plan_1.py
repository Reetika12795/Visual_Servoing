
import math
import time
import matplotlib.pyplot as plt
# from get_aruco_coord import *
import rospy
from geometry_msgs.msg import PoseStamped,Twist
from transformation_m import *

class Robot:
    def __init__(self,name,max_speed , max_rot_speed,init_pos):
        self.__x = init_pos[0]
        self.__y = init_pos[1]
        self.__theta = init_pos[2]* math.pi/180
        self.__name = name

        self.__max_speed = max_speed
        self.__max_rot_speed = max_rot_speed


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
        if(rotational_speed > self.__max_rot_speed):
            self.__rotational_speed = self.__max_rot_speed
        elif(rotational_speed <  - self.__max_rot_speed):
            self.__rotational_speed =  - self.__max_rot_speed
        else:
            self.__rotational_speed = rotational_speed
    def get_rotational_speed(self):
        return self.__rotational_speed



class Controller:
    def __init__(self, forward_speed_gain, rotational_speed_gain,parking_gain):
        self.forward_speed_gain = forward_speed_gain
        self.rotational_speed_gain = rotational_speed_gain
        self.parking_gain = parking_gain

    def robot_orientation(self, current_position, target_position): #returns the forward and rotational speed
        difference = [(target_position[0] - current_position[0]), (target_position[1] - current_position[1])]
        theta = target_position[2] * (math.pi/180)
        alpha = math.atan2(  target_position[1] ,target_position[0] )
        beta = theta - alpha
        self.rotationalspeed = self.rotational_speed_gain * float(alpha) + self.parking_gain * float(beta)
        print("beta:", beta*(180/math.pi),"\n")
        print("alpha:", alpha*(180/math.pi),"\n")
        a = [difference[0]*self.forward_speed_gain, difference[1]*self.forward_speed_gain]
        self.forwardspeed = math.sqrt((a[0])**2 + (a[1])**2) #resultant of x and y linear components speed
        return (self.forwardspeed , self.rotationalspeed)

#distance from two points
def distance_to_target(current_position, target_position):
    distance = math.sqrt((target_position[0]-current_position[0])**2 + (target_position[1]-current_position[1])**2 )
    return distance

initial = [0,0,0]


rospy.init_node('scan_move_node')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
rate = rospy.Rate(50)
robot = Robot("Jimmy", 0.08,0.4, initial)
controller = Controller(0.5, 0.8,-0.3)
iteration_time_sec = 0.01
robot.set_dtime(iteration_time_sec)

t_k = time.time()
final_time = time.time()
k=1
# dl = tb3_lidar()
# lidar_array = dl.data_return()
dist_front = 0.4
dist_rt_lft = 0.14
# print(f"{lidar_array}")
while True:
    # fwl, fwd, fwr = return_lidar_data()
    # print(f'{fwl = }')
    # print(f'{fwd = }')
    # print(f'{fwr = }')

    trans_final = get_final_tran()
    target_pos = get_Target_inRobot_frame(trans_final)
    pos = robot.get_position()[:2]
    # theta = get_Target_inRobot_frame(trans_final)[2]
    distance = distance_to_target(robot.get_position()[:2], target_pos)

    if distance < 0.2:
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
        #robot.update_position()
        print(f"step :{k} \n forwardspeed = ", robot.get_forward_speed(),"r_ speed : ",
        robot.get_rotational_speed(), " current position :" , robot.get_position())
        print("target pose", target_pos)
        # rostopic pub mobile_base_controller/cmd_vel geometry_msgs/Twist -r 3 -- '[0.5,0.0,0.0]' '[0.0, 0.0, 0.0]'
        forwards = robot.get_forward_speed()
        rotations = robot.get_rotational_speed()
        twist = Twist()

        

        if fwl < dist_rt_lft and fwl != 0.0:
            twist.linear.x = 0
            twist.linear.y = 0
            twist.angular.z = -0.8

        if fwd > dist_front and fwd != 0.0:
            print(f"{fwd = }")
            twist.linear.x = forwards
            twist.linear.y = forwards
            twist.angular.z = rotations

        if fwd < dist_front and fwd != 0.0:
            print(f"{fwd = }")
            twist.linear.x = 0
            twist.linear.y = 0
            twist.angular.z = 0.8
            

        if fwr < dist_rt_lft and fwr != 0.0:
            twist.linear.x = 0
            twist.linear.y = 0
            twist.angular.z = 0.8



#         twist.linear.x = forwards
#         # print(f'{twist.linear.x =}')
#         # twist.linear.x=0.1
#         twist.linear.y = forwards
#         # print(f'{twist.linear.y =}')
#         # twist.linear.y=0
#         twist.linear.z =0
#         twist.angular.x = 0
#         twist.angular.y = 0
#         twist.angular.z = rotations
#         # twist.angular.z=0


        pub.publish(twist)
        print('Distance', distance)
        plt.plot(robot.get_position()[0], robot.get_position()[1],'g^')# shows the position of the robot (X,Y) at each iteration
        final_time = time.time()
        k = k+1
        
print("{} arrived to the target!".format(robot.get_name()))
# plt.show()
