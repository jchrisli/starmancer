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
        #defaultViewDist = max(550.0, self._fFpv * sizeHalfHeight / (self._fpvRes[1] / 3))
        defaultViewDist = self._fFpv * max(size3d, sizeHalfHeight) / (self._fpvRes[1] / 3.333)
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

    #def get_voi_fpv_projection(self, voi):
    #    #voi = self.get_voi(voiId)
            ## project this voi onto the fpv camera view
    #    em = self._camParams.get_ext_mat_fpv()
    #    im = self._camParams.get_int_mat_fpv()
    #    pos3d = voi['position3d']
    #    #pos3d = np.array([0, 0, 1000])
    #    size3d = voi['size3d']
    #    sizehh = voi['sizehh']
    #    voiId = voi['id']
    #    pos3dInFpv = em.dot(np.append(pos3d, [1.0]).reshape(4, 1))
    #    dist2Fpv = pos3dInFpv[2, 0]
    #    if dist2Fpv <= 0:
    #        return None
    #    widthInFpv = 2 * self._fFpv * size3d / dist2Fpv
    #    heightInFpv = 2 * self._fFpv * sizehh / dist2Fpv
    #    posInFpv = im.dot(pos3dInFpv)
    #    posInFpv = (posInFpv / posInFpv[2, 0]).flatten()
    #    if self.__in_fpv_frame(posInFpv[0], posInFpv[1]):
    #        return (voiId, posInFpv.tolist(), widthInFpv, heightInFpv)
    #    else:
    #        return None

    def get_voi(self, id):
        voi = filter(lambda v: v["id"] == id, self.vois)
        if len(voi) > 0:
            return voi[0]
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
        bb = [(42.73686555, 715.24277647, 150 + self._CAM_DIAG / 2.0),\
            (-1647.58799028, 1052.11543762, 150 + self._CAM_DIAG / 2.0),\
            (-62.80333094, -1281.93800245, 150 + self._CAM_DIAG / 2.0)]
        return bb

