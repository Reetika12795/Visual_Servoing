import numpy as np
import math

def get_M(pose):

    theta = pose[2]*math.pi/180

    M = np.array([[math.cos(theta), -math.sin(theta),0 , pose[0]],

    [math.sin(theta), math.cos(theta),0,  pose[1]],

    [0,0,1,0],
    [0,0,0,1]])

    return M


def get_inR_fromTarget(M,pose):

    return M.dot(pose)

def get_inTarget(Mr, Mt):
    final_inverse = np.linalg.inv(Mr)
    trans_final = np.dot(final_inverse,Mt)

    return trans_final

pose_robot = [6,6,90,1]
pose_target= [18,6,90,1]

Mr = get_M(pose_robot)
Mt = get_M(pose_target)

final_m = get_inTarget(Mr,Mt)
print(final_m)


