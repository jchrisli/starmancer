"""
Controll One Tello
"""
from tello import Tello
import threading
#import cv2
import time
import json
from networking.viconConnection import ViconConnection
#from flightcontrollers.distance_flight_controller import DistanceFlightController
#from flightcontrollers.velocity_flight_controller import VelocityFlightController
from flightcontrollers.velocity_flight_controller_con import VelocityFlightControllerCon
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
# from pyquaternion import Quaternion
from utils.geometryUtils import vec_to_mat
from utils.motion_filter import Freq, LowPassDynamicFilter

class TelloController():
    def __init__(self, tello, debugUI = False):
        self.viewpointName = "FunkyTello"
        self.humanName = "Head1"

        self.camParams = CameraParameters()
        self.voiCalc = VoiCalulator(self.camParams)
        self.voiMng = VoiManager(self.camParams)

        self.subgoalAccessLock = threading.Lock()
        self.actionPlan = ActionPlanner(self.subgoalAccessLock, self.voiMng)
        self.state = []
        # Flight controller and its parameters
        # self.controller = DistanceFlightController(self.actionPlan)
        self.controller = VelocityFlightControllerCon()
        self.actionPlan.add_plan_subscriber(self.controller)
        self._controller_active = True
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
        self.debugUI = debugUI
        #self.right_click_count = 0

        # Tracking
        self.vicon_connection = ViconConnection(self.handle_vicon_data)
        self.local_heading = np.array([0, 1, 0])
        # Tracking -- filtering
        self.vicon_freq = Freq()
        self.vicon_freq.set(45)
        self.position_filter = LowPassDynamicFilter(self.vicon_freq)
        self.rotation_filter = LowPassDynamicFilter(self.vicon_freq)

        self.position_filter.set_cutoff_freq_low(5)
        self.position_filter.set_cutoff_freq_high(5)
        self.position_filter.set_velocity_low(10)
        self.position_filter.set_velocity_high(10)

        self.rotation_filter.set_cutoff_freq_low(5)
        self.rotation_filter.set_cutoff_freq_high(5)
        self.rotation_filter.set_velocity_low(10)
        self.rotation_filter.set_velocity_high(10)
        
        self.fps_count = 0
        # User interfaces
        # self.web_server = UiWebServer(self.web_server_port)    

        # self.ws_server = UiWsHandler2(self.ws_port)

        self.tello = tello
        ## From control signal to actual rc commands sent to the drone
        self._CMD_SCALE = 100
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
        self._controller_ready = True
        self._controller_update_event = threading.Event()
        self._controller_update_thread = RecurringEvent(self._controller_update_event, self.__reset_controller_update_flag, 0.2)
        
        ## Manual control params
        self._MANUAL_DIST = 0.2 # Other APIs use meter as the unit
        self._MANUAL_DEG = 10
        self._home_pressed = 0
        self._home_position = [0, -1500, 1200]
        self._home_dir = [0, 1, 0]
        self.voiMng.set_home_voi(self._home_position)

        self._manual_timer = None
        self._in_manual = False
        self._oc_focus_id = -1
        self._oc_manual_command_count = { \
            'left': 0, \
            'right': 0, \
            'forward': 0, \
            'backward': 0, \
            'up': 0, \
            'down': 0 \
        }
        self._oc_manual_command_threshold = 3
        self._oc_manual_command_count_lock = threading.Lock()

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
                self.actionPlan.generate_subgoals_voi_onstilts(self.state[0:3], self.state[3:], home_voi, self._home_dir)
                rmall_cmd = {'type': 'removeall'}
                self.command_transport.send(json.dumps(rmall_cmd))
                ## Clear all existing vois
                self.voiMng.clear_all_vois()
        elif key == keyboard.Key.esc:
            self._esc_pressed += 1
            if self._esc_pressed > 1:
                self._esc_pressed = 0
                #if
                self.tello.land()
        
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
            move, look, tlIntersection, xl, yl = self.voiCalc.get_roi_top_ground_intersection(topRoi['Left'], topRoi['Right'], topRoi['Top'], topRoi['Bottom'])
            print('Get roi topdown')
            if move is None:
                self.actionPlan.generate_subgoals_look(self.state[0:3], self.state[3:], look.tolist())
            else:
                look_after_move = look - move
                look_after_move = look_after_move / np.linalg.norm(look_after_move)
                self.actionPlan.generate_subgoals_position(self.state[0:3], self.state[3:], move.tolist(), look_after_move.tolist())
            # self.__get_goal_for_controller()
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
                self.actionPlan.generate_subgoals_voi_onstilts(self.state[0:3], self.state[3:], lookAtVoi, lookDir)
                # self.__get_goal_for_controller()

        elif data['Type'] == 'focus':
            focusId = data['Id']
            print('Get focus update command for %s' % focusId)
            self._oc_focus_id = focusId # -1 for no focus

        elif data['Type'] == 'focus3d':
            focusId = data['Id']
            focusVoi = self.voiMng.get_voi(focusId)
            vdir = data['LookDir']
            vpoint = data['LookPoint']
            print('Get focus 3d command for %d %s %s' % (focusId, str(vdir), str(vpoint)))
            if focusVoi is not None:
                self._oc_focus_id = focusId
                self.actionPlan.generate_subgoals_voi_onstilts(self.state[0:3], self.state[3:], focusVoi, vdir, vpoint)
                # self.__get_goal_for_controller()

        elif data['Type'] == 'approach':
            focusId = data['Id']
            focusVoi = self.voiMng.get_voi(focusId)
            if focusVoi is not None:
                vdir = focusVoi['position3d'] - np.array(self.state[0:3])
                self.actionPlan.generate_subgoals_voi_onstilts(self.state[0:3], self.state[3:], focusVoi, vdir)
                # self.__get_goal_for_controller()
            
        elif data['Type'] == 'move':
            if self._oc_focus_id != -1:
                ## only respond to commands from one joystick in the object centric mode
                ## if both are present, use the data from right
                focusVoi = self.voiMng.get_voi(self._oc_focus_id)
                direction = data['Direction'][1] if data['Distance'][1] >= 0 else data['Direction'][0]
                if self._oc_manual_command_count[direction] > self._oc_manual_command_threshold:
                    if direction == 'up' or direction == 'down':
                        if abs((focusVoi['position3d'] - np.array(self.state[:3]))[2]) < focusVoi['sizehh']:
                            self.tello.rc(0, 0, 0.2 * (1 if direction == 'up' else -1) , 0)
                    #if direction == 'forward':
                    #    horizontal = np.append(focusVoi['position3d'][:2] - np.array(self.state[:3])[:2], [0], axis=0)
                    #    horizontal_dist = np.linalg.norm(horizontal)
                    #    print('horizontal dist, view dist, size3d: %s %s %s' % (horizontal_dist, focusVoi['view_dist'], focusVoi['size3d']))
                    #    if horizontal_dist > min(focusVoi['view_dist'] / 1.5, focusVoi['size3d'] + 200):
                    #        #TODO: calculate the velocity vector to voi center
                    #        self.tello.rc(0, 0.2, 0, 0)
                    #if direction == 'backward':
                    #    self.tello.rc(0, -0.2, 0, 0)

                    if self._manual_timer is not None:
                        self._manual_timer.cancel()
                    self._manual_timer = threading.Timer(0.5, self.__quit_manual)
                    self._manual_timer.start()
                else:
                    self._oc_manual_command_count[direction] += 1
                    if self._oc_manual_command_count[direction] > self._oc_manual_command_threshold:
                        self._oc_manual_command_count_lock.acquire()
                        for k in self._oc_manual_command_count.keys():
                            if k != direction:
                                self._oc_manual_command_count[k] = 0
                        self._oc_manual_command_count_lock.release()
                        focusVoi = self.voiMng.get_voi(self._oc_focus_id)
                        if not self._in_manual:
                            self._in_manual = True
                            if direction == 'left' or direction == 'right' or direction == 'forward' or direction == 'backward':
                            # First manual control command, generate 
                                self._controller_active = True
                                if direction == 'left' or direction == 'right':
                                    self.actionPlan.generate_subgoals_manual_orbit(self.state[0:3], self.state[3:], focusVoi, 'l' if direction == 'left' else 'r') 
                                else:
                                    self.actionPlan.generate_subgoals_manual_zoom(self.state[0:3], self.state[3:], focusVoi, 'i' if direction == 'forward' else 'o')
                            if direction == 'up' or direction == 'down':
                                # self.actionPlan.abort_subgoals()
                                self._controller_active = False
            # Pure manual control                    
            else:
                direction = data['Direction'] 
                angle = data['Angle']
                dist = data['Distance']
                roll = pitch = yaw =  thrust = 0
                if dist[0] > 0:
                    if direction[0] == 'up' or direction[0] == 'down':
                        thrust = dist[0] * (1 if direction[0] == 'up' else -1)
                    else:
                        yaw = dist[0] * (1 if direction[0] == 'cw' else -1)
                if dist[1] > 0:
                    ## Right/left
                    ## Turn joystick angle into Cartesian angle
                    ## (360)|(0)            |
                    ##      |       to _____|_____(0)
                    ## _____|______               (360)
                    angle_r = -angle[1] + 2 * np.pi + np.pi / 2.0 if -angle[1] + np.pi / 2.0 < 2 * np.pi else -angle[1] + np.pi / 2.0
                    roll = dist[1] * np.cos(angle_r)
                    pitch = dist[1] * np.sin(angle_r)
                self.tello.rc(roll, pitch, thrust, yaw)
                if not self._in_manual:
                    self._in_manual = True
                if self._controller_active:
                    self._controller_active = False
                if self._manual_timer is not None:
                    self._manual_timer.cancel()
                self._manual_timer = threading.Timer(0.5, self.__quit_manual)
                self._manual_timer.start()
           
    def __query_battery(self):
        print('Current battery level is %s' % self.tello.get_battery())
        # self.tello.send_command('command')

    def __reset_controller_update_flag(self):
        self._controller_ready = True
        ## also update vicon fps
        self.vicon_freq.set(self.fps_count * 1 / 0.2)
        self.fps_count = 0

    def __quit_manual(self):
        self._in_manual = False
        self._oc_manual_command_count_lock.acquire()
        for k in self._oc_manual_command_count.keys():
            self._oc_manual_command_count[k] = 0
        self._oc_manual_command_count_lock.release()
        self.actionPlan.abort_subgoals()
        self._controller_active = True

    def handle_vicon_data(self, data):
        self.fps_count += 1
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

            ### Update drone actor state
            #self.situ.get_actor('cam').set_position(translation)
            #self.situ.get_actor('cam').set_rotation(rotation)

            ## Filtering
            #translation = self.position_filter.apply(translation)

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
            # signal = self.controller.generate_control_signal(observation)
            ## Only calculate control signal every 300ms
            if self._controller_ready:
                self._controller_ready = False
                R = vec_to_mat(observation[3:])
                yaw_dir = R.dot(np.transpose(np.array([self.local_heading])))
                self.state = list(observation[:3]) + yaw_dir.flatten().tolist()
                if self._controller_active:
                    control_ret, signal = self.controller.odom_callback(observation, self.debugUI)
                        
                    if not self.debugUI:
                        if control_ret == 1:
                            # Took too long, mission aborted
                            print('Timeout or cancelled. Mission aborted.')
                            self.tello.rc(0, 0, 0, 0)
                            # self.tello.land()
                        else:
                            # Mission in progress
                            # print('Control signal is %s' % str(signal))
                            self.tello.rc(signal[0], signal[1], signal[2], -signal[3])
                    else:
                        # Took too long, mission aborted
                        if control_ret == 1:
                            # force task reload
                            self.controller.get_next_subgoal()

                        # self.__get_goal_for_controller()
                ## Convert mm to cm, as the API uses cm
                # signal_in_cm = [int(s / 10) for s in signal[:-1]] + [int(signal[-1])]

                #if len(signal_in_cm) == 5:
                #    ## control signal for line goal
                #    (pos_speed, x_input, y_input, z_input, yaw_input) = signal_in_cm
                    #if(self.debug):
                    #    self.i_pitch.append(yaw_input)
                    #    self.i_roll.append(y_input)
                #    if pos_speed != 0:
                #        self.tello.go(x_input, y_input, z_input, pos_speed)
                #else:
                #    (arc_speed, x1, y1, z1, x2, y2, z2, yaw_input) = signal_in_cm
                #    if arc_speed != 0:
                #        self.tello.curve(x1, y1, z1, x2, y2, z2, arc_speed)

                #if yaw_input > 0:
                #    self.tello.rotate_cw(yaw_input)
                #elif yaw_input < 0:
                #    self.tello.rotate_ccw(-yaw_input)
                    
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
            self._controller_update_thread.start()

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



        
