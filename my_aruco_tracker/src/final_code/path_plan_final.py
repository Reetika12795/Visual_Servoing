class Robot:
    def __init__(self, robot_name, max_speed, initial_pos):
        self.robot_name = robot_name
        if max_speed > 0.3:
            self.max_speed = 0.3
        else:
            self.max_speed = max_speed
        self.initial_pos = initial_pos
    
    def update_current_pos(self, initial_pos):
        self.initial_pos = initial_pos

import time
import math

iteration_time_sec = 1000
target_pos = (1, 3)
initial_pos = (0, 0)
current_pos = initial_pos
t_k= time.time()
robot_name = "Robo"
robot = Robot (robot_name, 0.3, initial_pos)
#robot.init_task(t_k)
current_time = time.time()
current_theta = 0
iteration_speed = 0.001

def distance_to_target (pos1, pos2):
    return (((pos1[0] - pos2[0])**2) + ((pos1[1] - pos2[1])**2))**(1/2)

class Controller:
    def __init__(self, forward_speed_gain, rotational_speed):
        self.forward_speed_gain = forward_speed_gain
        if rotational_speed > 0.3:
            self.rotational_speed = 0.3
        else:
            self.rotational_speed = rotational_speed
    
    def move_forward(self, current_pos, theta, iteration_speed):
        return float("{:.10f}".format(current_pos[0] + 
        self.forward_speed_gain*iteration_speed*math.cos(theta),2)),\
        float("{:.10f}".format(current_pos[1] + self.forward_speed_gain*iteration_speed*math.sin(theta)))
    
    def rotate(self, theta, current_theta):
        if abs(theta - current_theta) <= 0.3:
            return theta
        elif (theta - current_theta) > 0.3:
            return current_theta + 0.3
        return current_theta - 0.3





def getAngleRad(p1, p2):
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])

theta = getAngleRad(initial_pos, target_pos)

control = Controller(1, theta)
# print(control.move_forward(initial_pos))

start_time = time.time()
while True:
    distance = distance_to_target(current_pos, target_pos)
    if distance < 0.01:
        break
    else:
        # Compute forward and rotation speed with controller
        if theta != current_theta:
            print("Rotating Robo", end = " ")
            current_theta = control.rotate(theta, current_theta)
            print("Current theta is at: ", current_theta, ", needed to ", theta)
        
        else:
            time.sleep(iteration_speed)
            print("Moving Forward", end = " ")
            current_pos = control.move_forward(current_pos, current_theta, iteration_speed)
            print("Distance is: ",distance, "Current location is at: ", current_pos, ", needed to ", target_pos)

print(time.time() -  start_time)