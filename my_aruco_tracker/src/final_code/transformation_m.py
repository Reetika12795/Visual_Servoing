import math
import sys
import numpy as np
import time
from get_aruco_coord_new import *



def rodrigues_vec_to_rotation_mat(rodrigues_vec):
    theta = np.linalg.norm(rodrigues_vec)
    if theta < sys.float_info.epsilon:              
        rotation_mat = np.eye(3, dtype=float)
        #print(type(rotation_mat),'if')
    else:
        r = rodrigues_vec / theta
        I = np.eye(3, dtype=float)
        r_rT = np.array([
            [r[0]*r[0], r[0]*r[1], r[0]*r[2]],
            [r[1]*r[0], r[1]*r[1], r[1]*r[2]],
            [r[2]*r[0], r[2]*r[1], r[2]*r[2]]
        ])
        r_cross = np.array([
            [0, -r[2], r[1]],
            [r[2], 0, -r[0]],
            [-r[1], r[0], 0]
        ])
        rotation_mat = math.cos(theta) * I + (1 - math.cos(theta)) * r_rT + math.sin(theta) * r_cross
        #print(type(rotation_mat),'else')
    return rotation_mat 


def get_final_tran():
    pp = path_planning()
    rodrigues_vec, translational = pp.data_return()
    print(rodrigues_vec, translational, '1')
    pp.printing()
    # print('Move robot to a new position.')



    rot = rodrigues_vec_to_rotation_mat(rodrigues_vec)
    #rot2 = rodrigues_vec_to_rotation_mat(rodrigues_vec2)

    # translational = np.array([[0.12841935455799103],[0.7663044333457947],[2.2687675952911377]])
    # translational2 =np.array([[0.20283901691436768],[0.13371101021766663],[2.263547420501709]])
    arr = np.array([[0,0,0,1]])
    rot_tran = np.append(rot,translational,axis = 1)
    #rot_tran2 = np.append(rot2,translational2,axis = 1)

    final = np.append(rot_tran,arr, axis=0)
    #final2 = np.append(rot_tran2,arr, axis=0)
    # print("final =", final)

    final2 = np.array([[ 0.98420651, -0.02846756, -0.1747202 ,  0.01014867],
 [-0.12981157 , 0.55499721, -0.82166116 , 0.10305589],
 [ 0.12035991 , 0.83136496 , 0.54253645 , 2.26156569],
 [ 0.   ,       0.     ,     0.      ,    1.        ]])
    # print('final2 =',final2)

    final_inverse = np.linalg.inv(final)
    trans_final = np.dot(final_inverse,final2)
    print(trans_final)
    return trans_final

get_final_tran()
