import numpy as np
from pyquaternion import Quaternion

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

def point_to_line_dist(x1, x2, c):
    ## The line is vec_x1 + t * (vec_x2 - vec_x1)
    ## The point is vec_c
    ## From https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    n = x2 - x1
    n = n / np.linalg.norm(n)
    half_bottom_side = ((x1 - c).dot(n)) * n 
    dist = (x1 - c) - half_bottom_side
    return np.linalg.norm(dist)

def tangent_line_circle(p, c, r):
    pc = c - p
    pc_dist = np.linalg.norm(pc)
    pc_dir = pc / pc_dist
    pc_dir_col = np.reshape(pc_dir, (2, 1))
    if pc_dist <= r:
        return None
    else:
        theta = np.arcsin(r / pc_dist)
        s_theta = np.sin(theta)
        c_theta = np.cos(theta)
        rot_m1 = np.array([[c_theta, s_theta], [s_theta, c_theta]])
        rot_m2 = np.array([[c_theta, -s_theta], [-s_theta, c_theta]])
        tvec1 = rot_m1.dot(pc_dir_col).flatten()
        tvec2 = rot_m2.dot(pc_dir_col).flatten()
        return (tvec1, tvec2)

def line_seg_intersect_circle(c, r, p1, p2):
    x = c
    path_seg = p2 - p1
    path_seg_len = np.linalg.norm(path_seg)
    path_dir = path_seg / path_seg_len
    t = path_dir.dot(x - p1)
    closest = p1 + t * path_dir
    dist = np.linalg.norm(closest - x)
    if r >= dist:
        ## Check if any of the two intersection is on the path segment
        half = np.sqrt(r ** 2 - dist ** 2)
        #intersect_1 = closest + path_dir * half
        #intersect_2 = closest - path_dir * half
        intersect_1 = t + half
        intersect_2 = t - half
        return any([v < path_seg_len and v > 0 for v in [intersect_1, intersect_2]])
    else:
        return False

class CameraParameters():
    def __init__(self):
        self._intMatFpv = np.array([[885.5839394885504, 0.0, 488.2530454326725], \
                            [0.0, 883.6668510262632, 362.2197785226864], \
                            [0.0, 0.0, 1.0]])
        # 1280 / 400 = 3.2
        ratio = 1280.0 / 400
        self._intMatTop = np.array([[788.9962872461164 / ratio, 0.0, 627.0971100621931 / ratio ], \
                                    [0.0, 790.378055853906 / ratio, 345.54398722442954 / ratio], \
                                    [0.0, 0.0, 1.0]])
        ## This is someting 3 * 4 and constant
        #self._extMatTopR = np.array([[0, 1, 0], \
        #                            [1, 0, 0], \
        #                            [0, 0, -1]])
        #self._extMatTopT = np.array([[400.0], [0.0], [2600.0]])
        self._extMatTopR = np.array([[0.0178797,  -0.99970006, -0.01673636], [-0.76892453, -0.00304895, -0.63933229], [0.6390895, 0.02430007, -0.76874841]])
        self._extMatTopT = np.array([[-112.05672, -162.7593, 3789.6638]]).T
        self._extMatTop = np.hstack((self._extMatTopR, self._extMatTopT))
        self._matTop = self._intMatTop.dot(np.hstack((self._extMatTopR, self._extMatTopT)))
        self._extViconToCamera = np.array([1, 0, 0, 0, 0, -1, 0, 1, 0]).reshape(3, 3, order='F')
        camTiltAngle = -12.0 * np.pi / 180
        self._qRegCam = Quaternion(axis = [1.0, 0.0, 0.0], angle = camTiltAngle)
        self._extViconToCamera = self._extViconToCamera.dot(self._qRegCam.rotation_matrix)
        ## This is someting constantly changing
        self._extMatFpvR = None
        self._extMatFpvT = None
        self._extMatFpv = None
        self._matFpv = None
        self._vecUpWorld = None
        self._vecLookWorld= None
        self._fpvTrans = None
        #self._vecLookLocal = (self._qRegCam.inverse).rotation_matrix.dot(np.array([0, 1.0, 0]))
        #self._vecUpLocal= (self._qRegCam.inverse).rotation_matrix.dot(np.array([0, 0, 1.0]))
        self._vecLookLocal = self._qRegCam.rotation_matrix.dot(np.array([0, 1.0, 0]))
        self._vecUpLocal= self._qRegCam.rotation_matrix.dot(np.array([0, 0, 1.0]))

        self._fFpv = (self._intMatFpv[0, 0] + self._intMatFpv[1, 1]) / 2
        self._fTop = (self._intMatTop[0, 0] + self._intMatTop[1, 1]) / 2
        self._fpvRes = (768.0, 576.0)
        self._fov = 57 / 180.0 * np.pi

        self._distFpv = np.array([-0.04219301378626456, -0.02323337060145524, -0.0022581736726784, \
            0.0007159763745722206, 0.1630269567794566], np.float32)
        self._distTop = np.array([0.07898490583963713, -0.050094270025822, -0.006732628640991619, \
                                0.0014357674069558882, -0.20715099275443166], np.float32)

    '''
        World -> Vicon -> Camera
        inv(R_wv) * inv(R_vc) - inv(R_wv) * T

        TODO: see if there is threading issue here
    '''
    def update_fpv_ext(self, rot, trans):
        #self._extMatFpv = np.concatenate((rot, trans), axis=1)
        rot = np.array(rot).reshape(3, 3)
        ## Update camera up and look vector in the world space
        self._vecLookWorld = rot.dot(self._vecLookLocal)
        self._vecUpWorld = rot.dot(self._vecUpLocal)
        rot = rot.dot(self._extViconToCamera)
        self._fpvTrans = np.array(trans)
        trans = self._fpvTrans.reshape(3, 1)
        try:
            self._extMatFpvR = np.linalg.inv(rot)
            self._extMatFpvT = - self._extMatFpvR.dot(trans)
            self._extMatFpv = np.hstack((self._extMatFpvR, self._extMatFpvT))
            ## Update fpv camera calibration matrix
            self._matFpv = self._intMatFpv.dot(self._extMatFpv)
        except np.linalg.LinAlgError:
            ## Yes, just silently pass
            pass
    
    def get_mat_top(self):
        return self._matTop

    def get_int_mat_top(self):
        return self._intMatTop

    def get_int_mat_fpv(self):
        return self._intMatFpv

    def get_ext_mat_top(self):
        return self._extMatTop

    def get_ext_mat_top_inv(self):
        return np.linalg.inv(np.vstack((self._extMatTop, np.array([[0, 0, 0, 1]]))))

    def get_top_cam_position(self):
        return self.get_ext_mat_top_inv()[:3, 3].flatten()

    def get_fpv_cam_position(self):
        return self._fpvTrans
    
    def get_mat_fpv(self):
        return self._matFpv

    def get_f_top(self):
        return self._fTop

    def get_f_fpv(self):
        return self._fFpv
        
    def get_ext_mat_fpv(self):
        return self._extMatFpv

    def get_fpv_res(self):
        return self._fpvRes

    def get_look_vec(self):
        return self._vecLookWorld

    def get_up_vec(self):
        return self._vecUpWorld

    ## TODO: calculate using internal matrix
    def get_fov(self):
        return self._fov

    def get_fpv_dist(self):
        return self._distFpv

    def get_top_dist(self):
        return self._distTop