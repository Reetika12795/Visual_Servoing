import math
import sys
import numpy as np
import time
from get_aruco_coord_new import *
from tf.transformations import euler_from_quaternion
theta = 0.0
pp = path_planning()
  
#get target and the direction of the target from the transformation matrix
def get_Target_inRobot_frame(M):
    if M.shape == (3,3):
        x = M[0][2]
        y = M[1][2]
        #arctang of 
        theta = math.atan2(M[1,0], M[0,0]) *180/math.pi

    else: 
        x = M[0][3]
        y = M[1][3]
        theta = rotationMatrixToEulerAngles(M[:3,:3])[2]*180/math.pi #return in degrees
        
    return [x,y,theta]


def rotationMatrixToEulerAngles(R) :

    sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

def eulerAnglesToRotationMatrix(theta) :

    R_x = np.array([[1, 0, 0 ],
                [0, math.cos(theta[0]), -math.sin(theta[0]) ],
                [0, math.sin(theta[0]), math.cos(theta[0]) ]
                ])
    
    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1]) ],
                    [0,                     1, 0 ],
                    [-math.sin(theta[1]),   0, math.cos(theta[1]) ]
                    ])
 
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
 
    R = np.dot(R_z, np.dot( R_y, R_x ))
 
    return R


def euler_angle(rot_vec):

    global theta
    # global pitch
    # (roll,pitch,theta) = euler_from_quaternion(rot_vec)
    [roll,pitch,theta] = euler_from_quaternion(rot_vec)
    # return [roll,pitch,theta] 
    return [roll,pitch,theta]

# def rodrigues_vec_to_rotation_mat(rodrigues_vec):
#     theta = np.linalg.norm(rodrigues_vec)
#     if theta < sys.float_info.epsilon:              
#         rotation_mat = np.eye(3, dtype=float)
#         #print(type(rotation_mat),'if')
#     else:
#         r = rodrigues_vec / theta
#         I = np.eye(3, dtype=float)
#         r_rT = np.array([
#             [r[0]*r[0], r[0]*r[1], r[0]*r[2]],
#             [r[1]*r[0], r[1]*r[1], r[1]*r[2]],
#             [r[2]*r[0], r[2]*r[1], r[2]*r[2]]
#         ])
#         r_cross = np.array([
#             [0, -r[2], r[1]],
#             [r[2], 0, -r[0]],
#             [-r[1], r[0], 0]
#         ])
#         rotation_mat = math.cos(theta) * I + (1 - math.cos(theta)) * r_rT + math.sin(theta) * r_cross
#         #print(type(rotation_mat),'else')
#     return rotation_mat 

def return_lidar_data():
    a,b,c = pp.data_return_lidar()

    return a,b,c

def get_final_tran():
    # pp = path_planning()
    rot_vec_rob, translational_vec_rob,rot_vec_target, translational_vec_target= pp.data_return()
    #print(rot_vec_rob, translational_vec_rob, 'robot')
    #print(rot_vec_target, translational_vec_target, 'target')
    #pp.printing()
    # print('Move robot to a new position.')


    rot_euler_rob = euler_angle(rot_vec_rob)
    #print("robot",rot_euler_rob)
    rot_euler_target = euler_angle(rot_vec_target)
    rot = eulerAnglesToRotationMatrix(rot_euler_rob)
    rot2 = eulerAnglesToRotationMatrix(rot_euler_target)
      
    #rot = rodrigues_vec_to_rotation_mat(rot_euler_rob)
    #rot2 = rodrigues_vec_to_rotation_mat(rot_euler_target)

    # translational = np.array([[0.12841935455799103],[0.7663044333457947],[2.2687675952911377]])
    # translational2 =np.array([[0.20283901691436768],[0.13371101021766663],[2.263547420501709]])
    arr = np.array([[0,0,0,1]])
    rot_tran = np.append(rot,translational_vec_rob,axis = 1)
    rot_tran2 = np.append(rot2,translational_vec_target,axis = 1)

    robot_tm= np.append(rot_tran,arr, axis=0)
    target_tm = np.append(rot_tran2,arr, axis=0)
    print("robot =\n", robot_tm)

#     target_tm = np.array([[-0.99970279 , 0.01174662 ,-0.02136221, -0.79320323],
#  [ 0.01057725 , 0.99848207  ,0.05405258 ,-0.13046248],
#  [ 0.02196472 , 0.05381056 ,-0.99830956 , 2.2350924 ],
#  [ 0.     ,     0.     ,     0.       ,   1.        ]])
    # target_tm = np.array([[-0.01203694 ,-0.99953618 ,-0.02797382 ,-0.79536134],
    #                     [-0.99930816  ,0.01104024 , 0.03551491 ,-0.13217212],
    #                     [-0.0351896  , 0.02838196 ,-0.99897756 , 2.43194866],
    #                     [ 0.       ,   0.       ,   0.       ,   1.        ]])
    # print('target =\n',target_tm)

    final_inverse = np.linalg.inv(robot_tm)
    #final2_inverse = np.linalg.inv(final2)
    # trans_final = np.dot(final_inverse,final2)
    trans_final = np.dot(final_inverse,target_tm)
    pos_robot = get_Target_inRobot_frame(trans_final)
    #theta_along_z = math.atan2(trans_final[1][3],trans_final[0][3]) *(180/math.pi)
    #print(f"{theta_along_z = }")
    #print(theta*(180/math.pi))
    #print(f"{pos_robot = }")
    # print('trans_final = \n',trans_final)
    return trans_final

get_final_tran()
