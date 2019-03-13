import sys
sys.path.insert(0, 'C:\Users\jiannanli\Research\drone\Tello-Python\Tello_Video')

from userinterfaces.VoiManager import VoiManager
from userinterfaces.CommandTransportUdp import CommandTransportUdp
import numpy as np
import json

vm = VoiManager()
transport = CommandTransportUdp(8091, 8092, lambda data: None)
transport.start()

def make_entry(x, y, z, r):
    voiC = np.array([x, y, z])
    voiR = r
    entry = vm.add_voi(voiC, voiR)
    # self.controller.approach(voiC.tolist(), voiR * 7)
    ## Send voi info to frontend
    ## Convert it to a list
    entry["position3d"] = entry["position3d"].tolist()
    entry["position_topdown"] = entry["position_topdown"].tolist()
    return entry

entry1 = make_entry(0.0, 0.0, 1000.0, 150.0)
transport.send(json.dumps(entry1))

entry2 = make_entry(-500.0, -1500.0, 0000.0, 150.0)
transport.send(json.dumps(entry2))
