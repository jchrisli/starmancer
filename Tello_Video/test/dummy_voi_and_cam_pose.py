import sys
sys.path.insert(0, 'C:\Users\jiannanli\Research\drone\Tello-Python\Tello_Video')

from userinterfaces.VoiManager import VoiManager
from userinterfaces.CommandTransportUdp import CommandTransportUdp
import numpy as np
import json
from utils.geometryUtils import CameraParameters

cp = CameraParameters()
vm = VoiManager(cp)
transport = CommandTransportUdp(8091, 8092, lambda data: None)
transport.start()

def make_entry(x, y, z, r, h):
    voiC = np.array([x, y, z])
    voiR = r
    voiHH = h
    entry = vm.add_voi(voiC, voiR, voiHH)
    # self.controller.approach(voiC.tolist(), voiR * 7)
    ## Send voi info to frontend
    ## Convert it to a list
    entry["position3d"] = entry["position3d"].tolist()
    entry["position_topdown"] = entry["position_topdown"].tolist()
    return entry

'''
Send voi positions
'''

entry1 = make_entry(0.0, 0.0, 1000.0, 100.0, 100.0)
addRoiCommand = {'type': 'roi3d', 'payload': entry1}
transport.send(json.dumps(addRoiCommand))

#entry2 = make_entry(-500.0, -1500.0, 0000.0, 150.0)
#transport.send(json.dumps(entry2))

'''
Update camera position once
'''
fpv_trans = [0, -1000.0, 1200]
#fpv_rot = [1, 0, 0, 0, 1, 0, 0, 0, 1]

camPosDict = {'type': 'cameraupdate', \
            'payload': {'Position': fpv_trans, \
                        'LookDir': [0, 1, 0], \
                        'UpDir': [0, 0, 1]}, \
            }
transport.send(json.dumps(camPosDict))