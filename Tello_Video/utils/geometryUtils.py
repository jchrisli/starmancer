import numpy as np

def euler_angle_to_mat(euler_x, euler_y, euler_z):
    sins = np.sin((euler_x, euler_y, euler_z))
    coss = np.cos((euler_x, euler_y, euler_z))
    sx = sins[0]
    sy = sins[1]
    sz = sins[2]
    cx = coss[0]
    cy = coss[1]
    cz = coss[2]

    mat = [[cy * cz, -cy * sz, sy],[
        cx * sz + sx * sy * cz, cx * cz - sx * sy * sz, -sx * cy], [
        sx * sz - cx * sy * cz, sx * cz + cx * sy * sz, cx * cy]]
    return mat

def vec_to_mat(nine_list):
    return np.array([[nine_list[0], nine_list[1], nine_list[2]], [
        nine_list[3], nine_list[4], nine_list[5]], [
            nine_list[6], nine_list[7], nine_list[8]]])
