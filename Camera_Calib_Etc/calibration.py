import numpy as np
import cv2
import glob
from VicionConnection import ViconConnection
import json
import yaml
import time

# # termination criteria
# # criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# # objp = np.zeros((6*7,3), np.float32)
# # objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
# # Arrays to store object points and image points from all the images.
# objpoints = [] # 3d point in real world space
# imgpoints = [] # 2d points in image plane.
# images = glob.glob('*.jpg')
# for fname in images:
#     img = cv.imread(fname)
#     gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
#     # Find the chess board corners
#     ret, corners = cv.findChessboardCorners(gray, (7,6), None)
#     # If found, add object points, image points (after refining them)
#     if ret == True:
#         objpoints.append(objp)
#         corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
#         imgpoints.append(corners)
#         # Draw and display the corners
#         cv.drawChessboardCorners(img, (7,6), corners2, ret)
#         cv.imshow('img', img)
#         cv.waitKey(500)
# cv.destroyAllWindows()

class FindExtrinsicInVicon():
    def __init__(self):
        self.model_name = 'Caliboard1'
        self.cycle = 50 ## Take a photo and find corners every <cycle> vicon packete 
        self.cycle_count = 0
        self.frame_needed = 20 ## Number of valid frames needed for calibration
        self.frame_count = 0
        self.grid_size = 23.5 ## mm 
        self.viconConn = ViconConnection(self._handle_vicon_data)
        self.board_size = (9, 6)
        self.objp_local = np.zeros((np.prod(self.board_size), 3), np.float32)
        #self.objp_local = np.zeros((1, 3), np.float32)
        #self.objp_local[0, 0] = self.grid_size
        #self.objp_local[0, 1] = self.grid_size
        combination = np.mgrid[1 : self.board_size[0] + 1, 1 : self.board_size[1] + 1].T.reshape(-1,2)
        self.objp_local[:, 0] = combination[:, 1] * self.grid_size 
        self.objp_local[:, 1] = combination[:, 0] * self.grid_size
        self.source = None
        self.objpoints = []
        self.imgpoints = []
        self.camera_matrix = np.array([[788.9962872461164, 0.0, 627.0971100621931], 
                                        [0.0, 790.378055853906, 345.54398722442954], 
                                        [0.0, 0.0, 1.0]], np.float32)
        self.dist_co = np.array([0.07898490583963713, -0.050094270025822, -0.006732628640991619, 
                                0.0014357674069558882, -0.20715099275443166], np.float32)
        
    def _handle_vicon_data(self, data):
        self.cycle_count = self.cycle_count + 1
        if self.cycle_count == self.cycle:
            self.cycle_count = 0
            start_time = time.time() 
            # First convert bytes to string
            dataStr = data.decode("utf-8")
            # To JSON 
            dataJ = json.loads(dataStr)
            name = dataJ["Name"]
            translation = dataJ["Translation"]
            transMat = np.tile(np.array(translation, np.float32).reshape(3, 1), np.prod(self.board_size))
            #transMat = np.array(translation, np.float32).reshape(3, 1)
            #transMat = np.array(translation, np.float32).reshape(1, 3)
            rotation = dataJ["Rotation"]
            rotMat = np.array(rotation, np.float32).reshape(3,3)
            #print str(transMat)
            if name == self.model_name:
                ## Calculate world position for the corners
                objp_world = (rotMat.dot(self.objp_local.T) + transMat).T
                #objp_world = transMat
                retval, img = self.source.read()
                if retval:
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    # Find the chess board corners
                    found, corners = cv2.findChessboardCorners(gray, self.board_size, cv2.CALIB_CB_FILTER_QUADS)
                    # If found, add object points, image points (after refining them)
                    if found == True:
                        #corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
                        self.frame_count = self.frame_count + 1
                        print 'Collecting frame {0}'.format(self.frame_count)
                        ## Save to debug folder
                        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                        cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)
                        #cv2.drawChessboardCorners(img, self.board_size, corners, found)
                        #cv2.imwrite('debug/%04d.png' % self.frame_count, img)
                        self.objpoints.append(objp_world)
                        #self.imgpoints.append(corners[0,:,:])
                        self.imgpoints.append(corners)
                        #print (time.time() - start_time) * 1000
                        #print str(objp_world) + str(corners[0,:,:])
                        if self.frame_count == self.frame_needed:
                            return False
        return True

    def Calibarte(self):
        print 'Starting camera ...'
        self.source = cv2.VideoCapture(1)
        #print "Frame default resolution: (" + str(self.source.get(cv2.CAP_PROP_FRAME_WIDTH)) + "; " + str(self.source.get(cv2.CAP_PROP_FRAME_HEIGHT)) + ")"
        self.source.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.source.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        print "Frame resolution: (" + str(self.source.get(cv2.CAP_PROP_FRAME_WIDTH)) + "; " + str(self.source.get(cv2.CAP_PROP_FRAME_HEIGHT)) + ")"

        print 'Starting Vicon listener ...'
        self.viconConn.start()
        print 'Waiting to collect Vicon data ...'
        self.viconConn.wait()
        print 'Computing extrinsic matrix ...'
        self.objpoints_array = np.array(self.objpoints, np.float32).reshape(-1, 3)
        self.imgpoints_array = np.array(self.imgpoints, np.float32).reshape(-1, 2)
        #self.objpoints_array = np.array(self.objpoints, np.float32)
        #self.imgpoints_array = np.array(self.imgpoints, np.float32)
        print 'Shape of world and image points {0} {1}'.format(self.objpoints_array.shape, self.imgpoints_array.shape)
        ret, rotVec, transVec, inliers = cv2.solvePnPRansac(self.objpoints_array, self.imgpoints_array, self.camera_matrix, self.dist_co)
        rotMat, jacobian = cv2.Rodrigues(rotVec)
        #calibration = {'ext_rot': str(rotMat), 'ext_trans': str(transVec), 'world_points': str(self.objpoints_array.tolist()), 'img_points': str(self.imgpoints_array.tolist())}
        calibration = {'ext_rot': str(rotMat), 'ext_trans': str(transVec)}
        with open('recent_topdown_calib.yaml', 'w') as fw:
            print 'Writing calibration results to file ...'
            yaml.dump(calibration, fw)


find = FindExtrinsicInVicon()
find.Calibarte()


