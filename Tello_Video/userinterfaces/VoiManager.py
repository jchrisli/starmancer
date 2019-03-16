'''
    Keep a list of defined volume of interest
'''

import numpy as np

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

    def add_voi(self, pos3d, size3d):
        voiId = self.nextId
        self.nextId = self.nextId + 1
        pos3dInTopdown = self._extMatTop.dot(np.append(pos3d, [1.0]).reshape(4, 1))
        # pos3dInTopdown = np.array([0, 0, 1000]).reshape(3, 1)
        #print('3D coords {0}'.format(str(pos3dInTopdown)))
        dist2Topdown = pos3dInTopdown[2, 0]
        sizeTopdown = self._fTop * size3d / dist2Topdown
        posInTopdown = self._intMatTop.dot(pos3dInTopdown)
        posInTopdown = (posInTopdown / posInTopdown[2, 0]).flatten()
        #print('2D coords {0}'.format(str(posInTopdown)))
        ## Divide by 3 to put the object in the center of the frame
        defaultViewDist = self._fFpv * size3d / (self._fpvRes[1] / 3)
        voiEntry = {"id": voiId, "position3d": pos3d, "size3d": size3d, "position_topdown": posInTopdown, "size_topdown": sizeTopdown, "view_dist": defaultViewDist}
        self.vois.append(voiEntry)
        return voiEntry

    def __in_fpv_frame(self, x, y):
        res = self._camParams.get_fpv_res()
        return x < res[0] and x > 0 and y < res[1] and y > 0

    def get_voi_fpv_projection(self, voi):
        #voi = self.get_voi(voiId)
            ## project this voi onto the fpv camera view
        em = self._camParams.get_ext_mat_fpv()
        im = self._camParams.get_int_mat_fpv()
        pos3d = voi['position3d']
        #pos3d = np.array([0, 0, 1000])
        size3d = voi['size3d']
        voiId = voi['id']
        pos3dInFpv = em.dot(np.append(pos3d, [1.0]).reshape(4, 1))
        dist2Fpv = pos3dInFpv[2, 0]
        if dist2Fpv <= 0:
            return None
        sizeInFpv = self._fFpv * size3d / dist2Fpv
        posInFpv = im.dot(pos3dInFpv)
        posInFpv = (posInFpv / posInFpv[2, 0]).flatten()
        if self.__in_fpv_frame(posInFpv[0], posInFpv[1]):
            return (voiId, posInFpv.tolist(), sizeInFpv)
        else:
            return None

    def get_voi(self, id):
        voi = filter(lambda v: v["id"] == id, self.vois)
        if len(voi) > 0:
            return voi[0]
        else:
            return None
    '''
        Return a list of 2-tuples that contain projected position and size of vois
        The elements of the list can be None
    '''
    def get_all_voi_projected(self):
        projected = map(lambda v: self.get_voi_fpv_projection(v), self.vois)
        projected = filter(lambda p: p is not None, projected)
        return projected

