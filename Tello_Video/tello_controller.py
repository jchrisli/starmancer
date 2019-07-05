"""
Controll One Tello
"""
from tello import Tello
#from DroneVision import DroneVision
import threading
#import cv2
import time
import json
from networking.viconConnection import ViconConnection
from flightcontrollers.distance_flight_controller import DistanceFlightController
# Debug
from pynput import keyboard
# from pynput.mouse import Listener, Button
# import pynput.keyboard
import matplotlib.pyplot as plt
#from userinterfaces.uiWsHandler2 import UiWsHandler2
#from userinterfaces.uiWebServer import UiWebServer
# from utils.transformations import Transformer
from utils.colorPrint import color_print 
from Situation import Situation
from utils.recurringEvent import RecurringEvent
from userinterfaces.CommandTransportUdp import CommandTransportUdp
from userinterfaces.VoiCalculator import VoiCalulator
from userinterfaces.VoiManager import VoiManager
import copy
from utils.geometryUtils import CameraParameters
from flightcontrollers.action_planner import ActionPlanner
import numpy as np
from pyquaternion import Quaternion

class TelloController():
    def __init__(self, tello):
        self.viewpointName = "FunkyTello"
        self.humanName = "Head1"

        self.camParams = CameraParameters()
        self.voiCalc = VoiCalulator(self.camParams)
        self.voiMng = VoiManager(self.camParams)

        self.subgoalAccessLock = threading.Lock()
        self.actionPlan = ActionPlanner(self.subgoalAccessLock, self.voiMng)
        # Flight controller and its parameters
        self.controller = DistanceFlightController(self.actionPlan)

        # Vicon connection
        self.vicon_connection = None
        # self.vicon_timestamp = 0

        # User interface WebSocket server
        #self.ws_server = None
        #self.ws_port = 8081
        # User interface html server
        #self.web_server = None
        #self.web_server_port = 8000

        self.command_transport_target_port = 8091
        self.command_transport_recv_port = 8092

        # thread for receving commands from the WPF frontend
        self.command_transport = CommandTransportUdp(self.command_transport_target_port, \
                                                    self.command_transport_recv_port, \
                                                    self._recv_command_handler) 

        self.control_sig = None
        self.state_x = []
        self.state_y = []
        self.i_pitch = []
        self.i_roll = []
        self.discrete_time = []
        self.start_time = None
        self.situ = Situation()

        # Debug
        self.debug = False
        self.debugUI = False
        #self.right_click_count = 0

        # Tracking
        self.vicon_connection = ViconConnection(self.handle_vicon_data)

        # User interfaces
        # self.web_server = UiWebServer(self.web_server_port)    

        # self.ws_server = UiWsHandler2(self.ws_port)

        self.tello = tello
        ## Check the connection status by querying tello status
        battery_level = self.tello.get_battery()
        if battery_level != 'none_response' or self.debugUI:
            self.success = True
        #self.success = True
        else:
            self.success = False
        
        ## Threading
        self._recur_query_event = threading.Event()
        self._recur_query_thread = RecurringEvent(self._recur_query_event, self.__query_battery, 3)

        ## Manual control params
        self._MANUAL_DIST = 0.2 # Other APIs use meter as the unit
        self._MANUAL_DEG = 10
        self._home_pressed = 0
        self._home_position = [0, -1500, 1200]
        self._home_dir = [0, 1, 0]
        self.voiMng.set_home_voi(self._home_position)

        self._esc_pressed = 0
        self._keyboard_list = keyboard.Listener(on_press=self.key_pressed)

        ## Record roi data onto a file
        timed_name = time.strftime('%d-%b-%H-%M-%S', time.localtime()) + '.csv'
        self._voi_file = open('voirecords/' + timed_name, 'wt')

        # rightnow = time.strftime('%H %M %S', time.localtime())
        # self._roi_history = open('roihistory_%s.txt' % rightnow, 'wt')

    '''
        def mouse_clicked(self, x, y, but, pressed):
            if but == Button.left:
                print('state is {0}'.format(str(self.controller.state)))
                print('control signal is {}'.format(str(self.control_sig)))
            elif but == Button.right and self.tello is not None:
                self.right_click_count = self.right_click_count + 1
                if self.right_click_count > 2:
                    self.tello.land()
                    self.right_click_count = 0
    '''

    '''
        Keyboard listener for manual control of the camera 
        W: up S: down A: rotate ccw D: rotate cw 
        Arrow up: forward Arrow down: backward Arrow left: left Arrow right: right
    '''
    def key_pressed(self, key):
        ## 
        print('%s pressed' % str(key))
        if key.char == 'w':
            self.tello.move_up(self._MANUAL_DIST)
        elif key.char == 's':
            self.tello.move_down(self._MANUAL_DIST)
        elif key.char == 'a':
            self.tello.rotate_ccw(self._MANUAL_DEG)
        elif key.char == 'd':
            self.tello.rotate_cw(self._MANUAL_DEG)
        elif key.char == 'i':
            self.tello.move_forward(self._MANUAL_DIST)
        elif key.char == 'k':
            self.tello.move_backward(self._MANUAL_DIST)
        elif key.char == 'j':
            self.tello.move_left(self._MANUAL_DIST)
        elif key.char == 'l':
            self.tello.move_right(self._MANUAL_DIST)
        ## Homing after pressing 'h' two times
        elif key.char == 'h':
            self._home_pressed += 1
            if self._home_pressed > 1:
                self._home_pressed = 0
                home_voi = self.voiMng.get_home_voi()
                self.actionPlan.generate_subgoals_voi_onstilts(self.controller.state[0:3], self.controller.state[3:], home_voi, self._home_dir)
                rmall_cmd = {'type': 'removeall'}
                self.command_transport.send(json.dumps(rmall_cmd))
                ## Clear all existing vois
                self.voiMng.clear_all_vois()
        elif key == keyboard.Key.esc:
            self._esc_pressed += 1
            if self._esc_pressed > 1:
                self._esc_pressed = 0
                if
        
    def _recv_command_handler(self, data):
        '''
            Listen to UDP packets sent to the port
            Calculate the volume of interest
        '''
        
        if data['Type'] == 'roi': # region of interest
            fpvRoi = data['FpvRoi']
            topRoi = data['TopdownRoi'] 
            self.voiCalc.set_2d_roi_fpv(fpvRoi['Left'], fpvRoi['Right'], fpvRoi['Top'], fpvRoi['Bottom'], fpvRoi['Type'])
            self.voiCalc.set_2d_roi_top(topRoi['Left'], topRoi['Right'], topRoi['Top'], topRoi['Bottom'], topRoi['Angle'], topRoi['Type'])
            ## TODO: there may be THREADING issues here
            (voiC, voiRTop, voiRFpv) = self.voiCalc.get_voi()
            print('Set voi at {0} with radius {1} and half height {2}'.format(str(voiC), str(voiRTop), str(voiRFpv)))
            self._voi_file.write(','.join([str(voiC[0]), str(voiC[1]), str(voiC[2]), str(voiRTop), str(voiRFpv)]))
            # self._roi_history.write('%s %s %s %s %s' % (voiC[0], voi[1], voi[2], voiRTop, voiRFpv))
            entry = self.voiMng.add_voi(voiC, voiRTop, voiRFpv)
            ## Send voi info to frontend
            ## Convert it to a list
            #entryCopy = json.loads(json.dumps(entry))
            entryCopy = copy.deepcopy(entry)
            entryCopy["position3d"] = entry["position3d"].tolist()
            entryCopy["position_topdown"] = entry["position_topdown"].tolist()
            addRoiCommand = {'type': 'roi3d', 'payload': entryCopy}
            self.command_transport.send(json.dumps(addRoiCommand))

        elif data['Type'] == 'roitopdown':
            topRoi = data['TopdownRoi']
            look, tlIntersection, xl, yl = self.voiCalc.get_roi_top_ground_intersection(topRoi['Left'], topRoi['Right'], topRoi['Top'], topRoi['Bottom'])
            print('Get roi topdown')
            self.actionPlan.generate_subgoals_look(self.controller.state[0:3], look.tolist())
            # self.controller.look_at(look)
            ## Notify the frontend to render a spotlight visualization
            spotlightPl = {'topleft_ground': tlIntersection.tolist(), 'x_length': xl, 'y_length': yl}
            spotlightCommand = {'type': 'spotlight', 'payload': spotlightPl}
            self.command_transport.send(json.dumps(spotlightCommand))
        
        elif data['Type'] == 'look':
            lookAtId = data['Id']
            print('Get look command for %s' % lookAtId)
            lookAtVoi = self.voiMng.get_voi(lookAtId)
            if lookAtVoi is not None:
                lookDir = [data['LookDir'][1], data['LookDir'][0], 0]
                self.actionPlan.generate_subgoals_voi_onstilts(self.controller.state[0:3], self.controller.state[3:], lookAtVoi, lookDir)

        ## This will be deprecated soon
        elif data['Type'] == 'focus':
            focusId = data['Id']
            print('Get focus command for %s' % focusId)
            focusVoi = self.voiMng.get_voi(focusId)
            if focusVoi is not None:
                ratioX = data['RatioX']
                # ratioY = data['RatioY'] ## ignore y axis for now
                camFacingDir = np.array(self.controller.state[3:])
                # camFacingDir = np.array(self.controller.state[3:])
                angleFromCenter = (ratioX - 0.5) * np.pi
                # print('Angle to rotate for focusing %s' % angleFromCenter)
                viewDir = -(Quaternion(axis = [0.0, 0.0, 1.0], angle = angleFromCenter).rotate(-camFacingDir))
                if ratioX > 0.75 or ratioX < 0.25:
                    self.actionPlan.generate_subgoals_voi_orbit(self.controller.state[0:3], self.controller.state[3:], focusVoi, viewDir)
                else:
                    self.actionPlan.generate_subgoals_voi_onstilts(self.controller.state[0:3], self.controller.state[3:], focusVoi, viewDir)

        elif data['Type'] == 'focus3d':
            focusId = data['Id']
            focusVoi = self.voiMng.get_voi(focusId)
            vdir = data['LookDir']
            vpoint = data['LookPoint']
            print('Get focus 3d command for %d %s %s' % (focusId, str(vdir), str(vpoint)))
            if focusVoi is not None:
                self.actionPlan.generate_subgoals_voi_onstilts(self.controller.state[0:3], self.controller.state[3:], focusVoi, vdir, vpoint)

        elif data['Type'] == 'approach':
            focusId = data['Id']
            focusVoi = self.voiMng.get_voi(focusId)
            if focusVoi is not None:
                vdir = focusVoi['position3d'] - np.array(self.controller.state[0:3])
                self.actionPlan.generate_subgoals_voi_onstilts(self.controller.state[0:3], self.controller.state[3:], focusVoi, vdir)
            

        elif data['Type'] == 'move':
            direction = data['Direction'] 
            if direction == 'forward':
                self.tello.move_forward(self._MANUAL_DIST)
            else: 
                self.tello.move_backward(self._MANUAL_DIST)
           

    def __query_battery(self):
        print('Current battery level is %s' % self.tello.get_battery())
        # self.tello.send_command('command')

    def handle_ws_message(self, action, data):
        if action == 'set_vp':
            x = data['x']
            y = data['y']
            z = data['z']
            yaw_x = data['yaw_x']
            yaw_y = data['yaw_y']
            yaw_z = data['yaw_z']
            # Transform from Unity space to Vicon space 
            #print('Received on WebSocket {0} {1} {2} {3} {4} {5}'.format(str(x), str(y), str(z), str(yaw_x), str(yaw_y), str(yaw_z)))
            #unity_pos = np.array([x, y, z, 1], dtype=np.float64)
            pos = [x, y, z]
            #vicon_pos = self.transformer.unity2ViconPt(unity_pos)
            #unity_dir = np.array([yaw_x, yaw_y, yaw_z, 1], dtype=np.float64)
            direction = [yaw_x, yaw_y, yaw_z]
            # color_print('Dir from Unity {0}'.format(str(unity_dir)), 'WARN')
            # vicon_dir = self.transformer.unity2ViconDir(unity_dir)
            # Calculate yaw from direction vector
        
            #target_position = vicon_pos + vicon_dir
            # color_print('Going to {0}'.format(str(target_position)), 'ERROR')
            target_position = pos + direction
            self.controller.set_target(target_position, time.time())
        elif action == 'local_vp':
            '''
             So the drone is going to take the viewpoint of the local person
            '''
            human_actor_vp = self.situ.get_anothers_viewpoint('human')
            self.controller.set_target(human_actor_vp, time.time())
            # color_print('Set local target at {0}'.format(human_actor_vp), type="WARN")
        elif action == 'set_vp_height':
            height = data['height']
            self.controller.set_target_z(height, time.time())
        elif action == 'land':
            self.tello.land()

    def __get_goal_for_controller(self):
        #with self.subgoalAccessLock:
        n = iter(self.actionPlan).next()
        self.controller.set_current_goal(n)
        if n is not None:
            target = n['params'][-6:]
            self.controller.set_target(target, time.time())

    def handle_vicon_data(self, data):
        curr_time = time.time()
        if (self.start_time is None):
            self.start_time = curr_time
            
        #if(curr_time - self.vicon_timestamp < 0.05 or not self.controller.target_set):
        #if(curr_time - self.vicon_timestamp < 0.05):
        #if(self.debugUI):
        #    return

        # First convert bytes to string
        dataStr = data.decode("utf-8")
        # print('Data received is %s' % dataStr)
        # To JSON 
        dataJ = json.loads(dataStr)
        name = dataJ["Name"]
        translation = dataJ["Translation"]
        rotation = dataJ["Rotation"]
        if name == self.viewpointName:
            '''
            # Do all transformation on the frontend
            # Transform the vicon position into unity position
            translation_unity = self.transformer.vicon2UnityPt(translation)
            rotation_unity = self.transformer.vicon2UnityRot(rotation)
            '''

            # Send through websocket to the frontend 
            #position_message = json.dumps({'type': 'current_vp', 'payload': {'translation': translation, 'rotation': rotation}})
            #if self.ws_server.started:
            #    self.ws_server.sendMessage(position_message)
            ### Update drone actor state
            self.situ.get_actor('cam').set_position(translation)
            self.situ.get_actor('cam').set_rotation(rotation)

            if(self.debug):
                # Record data later to be plotted
                self.discrete_time.append(curr_time - self.start_time)
                self.state_x.append(translation[0])
                self.state_y.append(translation[1])

            observation = tuple(translation + rotation)
            ## Update volume of interest calculator with camera position
            ## TODO: Note there might be threading issues here
            self.camParams.update_fpv_ext(rotation, translation)
            camPosDict = {'type': 'cameraupdate', \
                        'payload': {'Position': translation, \
                                    'LookDir': self.camParams.get_look_vec().tolist(), \
                                    'UpDir': self.camParams.get_up_vec().tolist()}, \
                        }
            self.command_transport.send(json.dumps(camPosDict))
            #print(json.dumps(camPosDict))

            ## Get updated projected voi position and send it to the frontend
            '''
            projected = self.voiMng.get_all_voi_projected()
            if projected is not None and len(projected) > 0:
                projectedArr = map(lambda p: {'id': p[0], 'position_fpv': p[1], 'w_fpv': p[2], 'h_fpv': p[3]}, projected)
                projectedDict = {'type': 'fpvupdate', 'payload': projectedArr}
                self.command_transport.send(json.dumps(projectedDict))
            '''
            signal = self.controller.generate_control_signal(observation)
                
            ''' TODO: do not send control signal if the drone is in emergency state (how to tell?) '''
            if not self.debugUI:
                # if all signals are None, we do nothing
                if (all([s is None for s in signal])):
                    if(self.debug):
                        self.i_pitch.append(0)
                        self.i_roll.append(0)
                else:
                    # print('Signal is %s' % str(signal))
                    ## Convert mm to cm, as the API uses cm
                    if all([s == 0 for s in signal]):
                        ## Subgoal achieved, fetch the next goal and set target
                        self.__get_goal_for_controller()
                        # self.vicon_timestamp = time.time()
                        return
                    signal_in_cm = [int(s / 10) for s in signal[:-1]] + [int(signal[-1])]

                    if len(signal_in_cm) == 5:
                        ## control signal for line goal
                        (pos_speed, x_input, y_input, z_input, yaw_input) = signal_in_cm
                        #if(self.debug):
                        #    self.i_pitch.append(yaw_input)
                        #    self.i_roll.append(y_input)
                        if pos_speed != 0:
                            self.tello.go(x_input, y_input, z_input, pos_speed)
                    else:
                        (arc_speed, x1, y1, z1, x2, y2, z2, yaw_input) = signal_in_cm
                        if arc_speed != 0:
                            self.tello.curve(x1, y1, z1, x2, y2, z2, arc_speed)

                    if yaw_input > 0:
                        self.tello.rotate_cw(yaw_input)
                    elif yaw_input < 0:
                        self.tello.rotate_ccw(-yaw_input)
                    
        elif name == self.humanName:
            '''
            Update human position through WebSocket
            '''
            # human_pos_msg = json.dumps({'type': 'current_hm', 'payload': {'translation': translation, 'rotation': rotation}})
            ### Update human actor state
            self.situ.get_actor('human').set_position(translation)
            self.situ.get_actor('human').set_rotation(rotation)
            # if self.ws_server.started:
            #    self.ws_server.sendMessage(human_pos_msg)
        # self.vicon_timestamp = time.time()

    def start(self):
        if (self.success or self.debugUI):
            #vicon_connection.wait()
            if not self.debugUI:
                print("taking off!")
                self.tello.takeoff()
                time.sleep(8)
                self.tello.move_up(0.2)
                ##self.tello.go(-15, -15, 0, 20)

            print("Preparing to open Vicon connection")
            self.vicon_connection.start()

            # print("Preparing to start web server")
            # self.web_server.startServer()

            #print("Preparing to open UI WebSocket connection")
            #self.ws_server.setMessageHandler(self.handle_ws_message)
            #self.ws_server.startServer()

            print("Preparing to open UI udp connection")
            self.command_transport.start()

            self._recur_query_thread.start()

            # self.mouse_listener = Listener(
            #                 on_move=None,
            #                 on_click=self.mouse_clicked,
            #                 on_scroll=None)
            self._keyboard_list.start()
            # self.mouse_listener.start()
            # target_position = [0, -2000, 1000, 0.0, 1.0, 0.0]
            # self.controller.set_target(target_position, time.time())
        else:
            print('Cannot get battery level. Check the connection.', 'ERROR')



        
