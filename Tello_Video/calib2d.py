'''
    Calculate the 2d vicon space to top-down camera view transformation matrix
    Input format: model_name1 model_name2 x_canvas_1 y_canvas_1 x_canvas_2 y_canvas_2
    Output format: transformation matrix in row major
'''

import sys
import numpy as np
import json

from networking.viconConnection import ViconConnection

class Calibrator():
    def __init__(self, model_name1, model_name2, model_name3, x_canvas_1, y_canvas_1, x_canvas_2, y_canvas_2, x_canvas_3, y_canvas_3):
        self.frames_to_collect = 300
        self.frames_collected = [0, 0, 0]
        self.model_x = [0, 0, 0]
        self.model_y = [0, 0, 0]
        self.model_names = [model_name1, model_name2, model_name3]
        self.canvas_pt = np.array([[float(x_canvas_1), float(x_canvas_2), float(x_canvas_3)], [float(y_canvas_1), float(y_canvas_2), float(y_canvas_3)], [1, 1, 1]])
        self.vicon_connection = ViconConnection(self.handle_vicon)
        self.vicon_connection.start()

    def handle_vicon(self, data):
        # First convert bytes to string
        dataStr = data.decode("utf-8")
        # To JSON 
        dataJ = json.loads(dataStr)
        name = dataJ["Name"]
        translation = dataJ["Translation"]
        if name in self.model_names:
            i = self.model_names.index(name)
            self.frames_collected[i] = self.frames_collected[i] + 1
            total_frames = sum(self.frames_collected)
            if total_frames  < self.frames_to_collect:
                self.model_x[i] = translation[0] + self.model_x[i] 
                self.model_y[i] = translation[1] + self.model_y[i]
            elif total_frames == self.frames_to_collect:
                vicon_pt = np.array([[self.model_x[0] / self.frames_collected[0], 
                                    self.model_x[1] / self.frames_collected[1],
                                    self.model_x[2] / self.frames_collected[2]],
                                    [self.model_y[0] / self.frames_collected[0],
                                    self.model_y[1] / self.frames_collected[1],
                                    self.model_y[2] / self.frames_collected[2]],
                                    [1, 1, 1]])
                ## CV_MAT * C = V
                ## VC_MAT * V = C
                cv_mat = vicon_pt.dot(np.linalg.inv(self.canvas_pt))
                vc_mat = self.canvas_pt.dot(np.linalg.inv(vicon_pt))
                print(cv_mat)
                print(vc_mat)

if len(sys.argv) != 10:
    print('please supply 6 parameters')
else:
    ## Listen on the udp port to get the position of the model for calibration
    model_name1 = sys.argv[1]
    model_name2 = sys.argv[2]
    model_name3 = sys.argv[3]
    x_canvas_1 = sys.argv[4]
    y_canvas_1 = sys.argv[5]
    x_canvas_2 = sys.argv[6]
    y_canvas_2 = sys.argv[7]
    x_canvas_3 = sys.argv[8]
    y_canvas_3 = sys.argv[9]
    cali = Calibrator(model_name1, model_name2, model_name3, x_canvas_1, y_canvas_1, x_canvas_2, y_canvas_2, x_canvas_3, y_canvas_3)
