import math
import sys
import numpy as np
import time
from get_aruco_coord import *
pp = path_planning()
rodrigues_vec, translational = pp.data_return()
print(rodrigues_vec, translational, '1')
time.sleep(10)
print('Move robot to a new position.')
rodrigues_vec2, translational2 = pp.data_return()
print(rodrigues_vec2, translational2, '2')
def rodrigues_vec_to_rotation_mat(rodrigues_vec):
    theta = np.linalg.norm(rodrigues_vec)
    if theta < sys.float_info.epsilon:              
        rotation_mat = np.eye(3, dtype=float)
        print(type(rotation_mat),'if')
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
        print(type(rotation_mat),'else')
    return rotation_mat 

rot = rodrigues_vec_to_rotation_mat(rodrigues_vec)
#rot2 = rodrigues_vec_to_rotation_mat(rodrigues_vec2)

# translational = np.array([[0.12841935455799103],[0.7663044333457947],[2.2687675952911377]])
# translational2 =np.array([[0.20283901691436768],[0.13371101021766663],[2.263547420501709]])
arr = np.array([[0,0,0,1]])
rot_tran = np.append(rot,translational,axis = 1)
#rot_tran2 = np.append(rot2,translational2,axis = 1)

final = np.append(rot_tran,arr, axis=0)
#final2 = np.append(rot_tran2,arr, axis=0)
print(final)

final2 = np.array([[ 0.95764711,  0.12385912 , 0.2599441 ,  0.62985319],
 [ 0.14182342  ,0.58277065 ,-0.80016529 , 0.16573983],
 [-0.25059556 , 0.80314213 , 0.54052251  ,2.26376104],
 [ 0.       ,   0.     ,     0.      ,    1.        ]])
print('final2 =',final2)

final2_inverse = np.linalg.inv(final2)
trans_final = np.dot(final,final2_inverse)
print(trans_final)