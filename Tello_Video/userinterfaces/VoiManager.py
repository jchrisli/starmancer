'''
    Keep a list of defined volume of interest
'''

import numpy as np
import math
import functools

class VoiManager():
    def __init__(self, camParams):
        '''
            each voi is a dictionary with keys "id", "position3d", "size3d", "position_topdown", "size_topdown"
        '''
        self.vois = []
        self.nextId = 0 
        self._camParams = camParams
        self._intMatTop = self._camParams.get_int_mat_top()
        self._extMatTop = self._camParams.get_ext_mat_top()
        self._fTop = self._camParams.get_f_top()
        self._intMatFpv = self._camParams.get_int_mat_fpv()
        self._fFpv = self._camParams.get_f_fpv()
        self._fpvRes = self._camParams.get_fpv_res()

        ## Add a virtual voi for the home position
        self._homeVoi = None
        self._CAM_DIAG = 134.76

    def set_home_voi(self, position):
        self._homeVoi = {'position3d': np.array(position), 'view_dist': 1}

    def get_home_voi(self):
        return self._homeVoi

    def add_voi(self, pos3d, size3d, sizeHalfHeight):
        voiId = self.nextId
        self.nextId = self.nextId + 1
        pos3dInTopdown = self._extMatTop.dot(np.append(pos3d, [1.0]).reshape(4, 1))
        # pos3dInTopdown = np.array([0, 0, 1000]).reshape(3, 1)
        #print('3D coords {0}'.format(str(pos3dInTopdown)))
        dist2Topdown = pos3dInTopdown[2, 0]
        # TODO: remove the topdown view related code
        sizeTopdown = self._fTop * size3d / dist2Topdown
        posInTopdown = self._intMatTop.dot(pos3dInTopdown)
        posInTopdown = (posInTopdown / posInTopdown[2, 0]).flatten()
        #print('2D coords {0}'.format(str(posInTopdown)))
        ## Put the object in the center of the frame
        # defaultViewDist = self._fFpv * max(size3d, sizeHalfHeight) / (self._fpvRes[1] / 3.333)
        defaultViewDist = self._fFpv * max(size3d, sizeHalfHeight) / (self._fpvRes[1] / 3.0)
        voiEntry = {"id": voiId, "position3d": pos3d, "size3d": size3d, "sizehh": sizeHalfHeight, "position_topdown": posInTopdown, "size_topdown": sizeTopdown, "view_dist": defaultViewDist}
        self.vois.append(voiEntry)
        return voiEntry

    def clear_all_vois(self):
        self.vois = []
        self.nextId = 0 

    def __in_fpv_frame(self, x, y):
        res = self._camParams.get_fpv_res()
        return x < res[0] and x > 0 and y < res[1] and y > 0
    
    ## TODO: test against a ellipsoid rather than a sphere
    def __test_line_seg_against_voi(self, voi, p1, p2):
        ## Only do the sphere case for now
        x = voi['position3d']
        r = voi['size3d']
        path_seg = p2 - p1
        path_seg_len = np.linalg.norm(path_seg)
        path_dir = path_seg / path_seg_len
        t = path_dir.dot(x - p1)
        closest = p1 + t * path_dir
        dist = np.linalg.norm(closest - x)
        # print('r and dist are %s %s' % (r, dist))
        if r >= dist:
            ## Check if any of the two intersection is on the path segment
            half = math.sqrt(r ** 2 - dist ** 2)
            #intersect_1 = closest + path_dir * half
            #intersect_2 = closest - path_dir * half
            intersect_1 = t + half
            intersect_2 = t - half
            return any([v < path_seg_len and v > 0 for v in [intersect_1, intersect_2]])
        else:
            return False

    def get_voi(self, id):
        voi = filter(lambda v: v["id"] == id, self.vois)
        if len(voi) > 0:
            return voi[0]
        else:
            return None

    def get_offset_for_transform(self, ind, scale, translate):
        voi = self.get_voi(ind)
        if voi is not None:
            ## Calculate current voi position in the camera frame  
            voi_pos_world = np.array(voi['position3d']).reshape(3, 1)
            voi_pos_world = np.append(voi_pos_world, np.array([[1]]), axis=0)
            ext_mat = self._camParams.get_ext_mat_fpv()
            R_inv = np.linalg.inv(ext_mat[:3,:3])
            voi_pos_cam_1 = ext_mat.dot(voi_pos_world)
            z1 = voi_pos_cam_1[2, 0]
            z2 = max(z1 / scale, voi['size3d'] + 150 + self._CAM_DIAG / 2.0)
            f = self._camParams.get_f_fpv()
            translate_arr = np.array(translate).reshape(2,1) + voi_pos_cam_1[:2, :] * f / z1
            denom = f / z2
            xy2 = translate_arr / denom
            xyz2 = np.append(xy2, np.array([[z2]]), axis=0)
            offset = R_inv.dot(voi_pos_cam_1[:3,:] - xyz2)
            print('Offset is %s' % offset.flatten())
            return offset.flatten()
        else:
            return None

    '''
        Return a list of 3-tuples that contain projected position and width and height of vois
        The elements of the list can be None
    '''
    #def get_all_voi_projected(self):
    #    projected = map(lambda v: self.get_voi_fpv_projection(v), self.vois)
    #    projected = filter(lambda p: p is not None, projected)
    #    return projected

    def test_path_against_all_voi(self, p1, p2):
        test_params_bound = functools.partial(self.__test_line_seg_against_voi, p1 = p1, p2 = p2)
        return any(map(test_params_bound, self.vois))

    """Get the dimension information of all vois, projected onto the x-y plane
    """

    def get2dvois(self):
        vois2d = [(v['position3d'][0], v['position3d'][1], v['size3d'] + self._CAM_DIAG / 2.0) for v in self.vois]
        return vois2d

    def get2d_bounding_boxex(self):
        bb = [(42.73686555, 715.24277647, 150 + self._CAM_DIAG / 2.0, 1042.1084, 326.8656),\
            (-1390.2046069, 836.44244155, 150 + self._CAM_DIAG / 2.0, 1280.83605, 231.01754999999991),\
            (-20.01757625, -1500.32728548, 150 + self._CAM_DIAG / 2.0,  1214.2033000000001, 166.32910000000004)]
        return bb
        
    def get_manual_collision_volume(self):
        bb = [(42.73686555, 715.24277647, 250 + self._CAM_DIAG, (326.8656 + 1042.1084) / 2, (326.8656 + 1042.1084) / 2),\
            (-1390.2046069, 836.44244155, 200 + self._CAM_DIAG, 1504.1146 / 2, 1504.1146 / 2),\
            (-20.01757625, -1500.32728548, 250 + self._CAM_DIAG,  1431.49 / 2, 1431.49 / 2)]
        return bb

