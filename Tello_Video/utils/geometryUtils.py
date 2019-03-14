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

class CameraParameters():
    def __init__(self):
        self._intMatFpv = np.array([[885.5839394885504, 0.0, 488.2530454326725], \
                            [0.0, 883.6668510262632, 362.2197785226864], \
                            [0.0, 0.0, 1.0]])
        self._intMatTop = np.array([[788.9962872461164 / 4, 0.0, 627.0971100621931 / 4 ], \
                                    [0.0, 790.378055853906 / 4, 345.54398722442954 / 4], \
                                    [0.0, 0.0, 1.0]])
        ## This is someting 3 * 4 and constant
        self._extMatTopR = np.array([[0, 1, 0], \
                                    [1, 0, 0], \
                                    [0, 0, -1]])
        self._extMatTopT = np.array([[400.0], [0.0], [2600.0]])
        self._extMatTop = np.hstack((self._extMatTopR, self._extMatTopT))
        self._matTop = self._intMatTop.dot(np.hstack((self._extMatTopR, self._extMatTopT)))
        self._extViconToCamera = np.array([1, 0, 0, 0, 0, -1, 0, 1, 0]).reshape(3, 3, order='F')
        ## This is someting constantly changing
        self._extMatFpvR = None
        self._extMatFpvT = None
        self._extMatFpv = None
        self._matFpv = None

        self._fFpv = (self._intMatFpv[0, 0] + self._intMatFpv[1, 1]) / 2
        self._fTop = (self._intMatTop[0, 0] + self._intMatTop[1, 1]) / 2
        self._fpvRes = (960, 720)

    def update_fpv_ext(self, rot, trans):
        #self._extMatFpv = np.concatenate((rot, trans), axis=1)
        rot = np.array(rot).reshape(3, 3)
        rot = self._extViconToCamera.dot(rot)
        trans = np.array(trans).reshape(3, 1)
        try:
            self._extMatFpvR = np.linalg.inv(rot)
            self._extMatFpvT = - self._extMatFpvR.dot(trans)
            self._extMatFpv = np.hstack((self._extMatFpvR, self._extMatFpvT))
            ## Update fpv camera calibration matrix
            self._matFpv = self._intMatFpv.dot(np.hstack((self._extMatFpvR, self._extMatFpvT)))
        except np.linalg.linalg.LinAlgError:
            ## Yes, just silently pass
            pass
    
    def get_mat_top(self):
        return self._matTop
    
    def get_mat_fpv(self):
        return self._matFpv

    def get_f_top(self):
        return self._fTop

    def get_f_fpv(self):
        return self._fFpv

    def get_int_mat_top(self):
        return self._intMatTop

    def get_int_mat_fpv(self):
        return self._intMatFpv

    def get_ext_mat_top(self):
        return self._extMatTop
        
    def get_ext_mat_fpv(self):
        return self._extMatFpv

    def get_fpv_res(self):
        return self._fpvRes