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
#from pynput import keyboard
from pynput.mouse import Listener, Button
import matplotlib.pyplot as plt
from userinterfaces.uiWsHandler2 import UiWsHandler2
from userinterfaces.uiWebServer import UiWebServer
# from utils.transformations import Transformer
from utils.colorPrint import color_print 
from Situation import Situation
from utils.recurringEvent import RecurringEvent

class TelloController():
    def __init__(self, tello):
        self.viewpointName = "FunkyTello"
        self.humanName = "Head1"

        # Flight controller and its parameters
        self.controller = DistanceFlightController()

        # Vicon connection
        self.vicon_connection = None
        self.vicon_timestamp = 0

        # User interface WebSocket server
        self.ws_server = None
        self.ws_port = 8080
        # User interface html server
        self.web_server = None
        self.web_server_port = 8000

        self.control_sig = None
        self.state_x = []
        self.state_y = []
        self.i_pitch = []
        self.i_roll = []
        self.discrete_time = []
        self.start_time = None
        self.situ = Situation()


        # Debug
        self.keyboard_listener = None
        self.debug = False
        self.debugUI = False
        self.right_click_count = 0

        # Tracking
        self.vicon_connection = ViconConnection(self.handle_vicon_data)

        # User interfaces
        self.web_server = UiWebServer(self.web_server_port)    

        self.ws_server = UiWsHandler2(self.ws_port)

        # self.transformer = Transformer()

        self.tello = tello
        ## Check the connection status by querying tello status
        battery_level = self.tello.get_battery()
        if battery_level != 'none_response':
            self.success = True
        #self.success = True
        else:
            self.success = False

        
        ## Threading
        self._recur_query_event = threading.Event()
        self._recur_query_thread = RecurringEvent(self._recur_query_event, self.__query_battery, 3)
        # self.success = self.mambo.connect(num_retries=3)
        # print("connected: %s" % self.success)

    def mouse_clicked(self, x, y, but, pressed):
        if but == Button.left:
            print('state is {0}'.format(str(self.controller.state)))
            print('control signal is {}'.format(str(self.control_sig)))
        elif but == Button.right and self.tello is not None:
            self.right_click_count = self.right_click_count + 1
            if self.right_click_count > 2:
                self.tello.land()
                self.right_click_count = 0
        

    def __query_battery(self):
        self.tello.get_battery() 

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

    def handle_vicon_data(self, data):
        curr_time = time.time()
        if (self.start_time is None):
            self.start_time = curr_time
            
        #if(curr_time - self.vicon_timestamp < 0.05 or not self.controller.target_set):
        #if(curr_time - self.vicon_timestamp < 0.05):
        if(not self.controller.target_set and not self.debugUI):
            return

        # First convert bytes to string
        dataStr = data.decode("utf-8")
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
            position_message = json.dumps({'type': 'current_vp', 'payload': {'translation': translation, 'rotation': rotation}})
            if self.ws_server.started:
                self.ws_server.sendMessage(position_message)
            ### Update drone actor state
            self.situ.get_actor('cam').set_position(translation)
            self.situ.get_actor('cam').set_rotation(rotation)

            if(self.debug):
                # Record data later to be plotted
                self.discrete_time.append(curr_time - self.start_time)
                self.state_x.append(translation[0])
                self.state_y.append(translation[1])

            observation = tuple(translation + rotation)
            signal = self.controller.generate_control_signal(observation)
            ''' TODO: do not send control signal if the drone is in emergency state (how to tell?) '''
            if not self.debugUI:
                # if all signals are zero, we let the quadrotor hover
                if (all([s is None for s in signal])):
                    if(self.debug):
                        self.i_pitch.append(0)
                        self.i_roll.append(0)
                else:
                    ## Convert mm to cm
                    [x_input, y_input, z_input, pos_speed, yaw_input] = [int(s / 10) for s in signal[:4]] + [int(signal[4])]
                    self.control_sig = [x_input, y_input, z_input, pos_speed, yaw_input]

                    if(self.debug):
                        self.i_pitch.append(yaw_input)
                        self.i_roll.append(y_input)
                    if pos_speed != 0:
                        self.tello.go(x_input, y_input, z_input, pos_speed)
                    if yaw_input > 0:
                        self.tello.rotate_cw(yaw_input)
                    elif yaw_input < 0:
                        self.tello.rotate_ccw(yaw_input)
                    
        elif name == self.humanName:
            '''
            Update human position through WebSocket
            '''
            human_pos_msg = json.dumps({'type': 'current_hm', 'payload': {'translation': translation, 'rotation': rotation}})
            ### Update human actor state
            self.situ.get_actor('human').set_position(translation)
            self.situ.get_actor('human').set_rotation(rotation)
            if self.ws_server.started:
                self.ws_server.sendMessage(human_pos_msg)
        self.vicon_timestamp = time.time()

    def start(self):
        if (self.success):

            if self.debugUI:
                return

            print("taking off!")
            #vicon_connection.wait()
            self.tello.takeoff()
            time.sleep(8)

            print("Preparing to open Vicon connection")
            self.vicon_connection.start()

            print("Preparing to start web server")
            self.web_server.startServer()

            print("Preparing to open UI WebSocket connection")
            self.ws_server.setMessageHandler(self.handle_ws_message)
            self.ws_server.startServer()

            self._recur_query_thread.start()

            self.mouse_listener = Listener(
                            on_move=None,
                            on_click=self.mouse_clicked,
                            on_scroll=None)
            self.mouse_listener.start()
            target_position = [0, 0, 1500, 0.0, 1.0, 0.0]
            self.controller.set_target(target_position, time.time())
        else:
            color_print('Cannot get battery level. Check the connection.', 'ERROR')



        
