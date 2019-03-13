'''
    Keep a list of defined volume of interest
'''

import numpy as np

class VoiManager():
    def __init__(self):
        '''
            each voi is a dictionary with keys "id", "position3d", "size3d", "position_topdown", "size_topdown"
        '''
        self.vois = []
        self.nextId = 0 

        self._intMatTop = np.array([[788.9962872461164 / 4, 0.0, 627.0971100621931 / 4 ], \
                            [0.0, 790.378055853906 / 4, 345.54398722442954 / 4], \
                            [0.0, 0.0, 1.0]])
        self._fTop = (self._intMatTop[0, 0] + self._intMatTop[1, 1]) / 2
        
        self._extMatTopT = np.array([[400.0], [0.0], [2600.0]])
        self._extMatTopR = np.array([[0, 1, 0], \
                            [1, 0, 0], \
                            [0, 0, -1]])
        self._extMatTop = np.hstack((self._extMatTopR, self._extMatTopT))
        self._matTop = self._intMatTop.dot(self._extMatTop)

        self._intMatFpv = np.array([[885.5839394885504, 0.0, 488.2530454326725], \
                            [0.0, 883.6668510262632, 362.2197785226864], \
                            [0.0, 0.0, 1.0]])
        self._fFpv = (self._intMatFpv[0, 0] + self._intMatFpv[1, 1]) / 2
        self.fpvRes = (960, 720)

    def add_voi(self, pos3d, size3d):
        voiId = self.nextId
        self.nextId = self.nextId + 1
        pos3dInTopdown = self._extMatTop.dot(np.append(pos3d, [1.0]).reshape(4, 1))
        #print('3D coords {0}'.format(str(pos3dInTopdown)))
        dist2Topdown = np.linalg.norm(pos3dInTopdown)
        sizeTopdown = self._fTop * size3d / dist2Topdown
        posInTopdown = self._intMatTop.dot(pos3dInTopdown)
        posInTopdown = (posInTopdown / posInTopdown[2, 0]).flatten()
        #print('2D coords {0}'.format(str(posInTopdown)))
        ## Divide by 3 to put the object in the center of the frame
        defaultViewDist = self._fFpv * size3d / (self.fpvRes[1] / 3)
        voiEntry = {"id": voiId, "position3d": pos3d, "size3d": size3d, "position_topdown": posInTopdown, "size_topdown": sizeTopdown, "view_dist": defaultViewDist}
        self.vois.append(voiEntry)
        return voiEntry
'''
    def get_voi_fpv_projection(self, voiId, pos3d, size3d, extFpv):
        voi = get_voi(voiId)
        if voi is not None:
'''

    def get_voi(self, id):
        voi = filter(lambda v: v["id"] == id, self.vois)
        if len(voi) > 0:
            return voi[0]
        else:
            return None

