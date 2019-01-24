''' position / attitude controller for the quadrotor
    Send distance flight commands
'''
import time
import numpy as np
from utils.geometryUtils import euler_angle_to_mat
from utils.geometryUtils import vec_to_mat
from numpy.linalg import inv
import math
import threading

class DistanceFlightController:
    # Initialize the controller
    # Params is a 8-tuple, (x_p, x_v, y_p, y_v, z_p, z_v, yaw_p, yaw_v)
    def __init__(self):
        self._target = [-1, -1, -1, -1, -1, -1] 
        self.target_set = False
        self._target_set_time = None
        ## x, y, z, yaw_x, yaw_y, yaw_z
        self.state = [-1, -1, -1, -1, -1, -1]
        ## velocity_x, velocity_y, velocity_z, velocity_yaw(in dg/second)
        self.diff_state = [-1, -1, -1, -1]
        self._last_tracked_observation = None # Record the last tracked frame for now 
        self.recent_control_sig = [0, 0, 0, 0]
        self.current_time = -1
        self._previous_time = -1

        self._last_timestamp = None
        # Local Vicon coordinate system to velocity
        self._align_mat = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        self._local_heading = np.array([0, 1, 0])

        ''' Parameters for distance-based control 
            All distance in milimeters
        '''
        self._dist_step_L = 1000
        self._dist_step_S = 283 ## sqrt(20^2*2)
        self._position_ready_dist = 370 ## sqrt(20^2*3)
        self._yaw_threshold = 15
        ## Do not move if speed is below the cap
        self._still_speed_cap = 100
        ## Cap for yaw
        self._still_yaw_speed_cap = 5
        self._speed_L = 300
        self._speed_S = 200
        ## Never send two commands within this interval
        self._minimum_signal_interval = 1
        self._within_interval = False
        self._interval_timer = None

    # Specify current target, a 6-tuple (x, y, z, head_dir_x, head_dir_y, head_dir_z)
    # For now our end-state is always hovering
    # The controller loop will try to track this target using PID controll
    def set_target(self, tar, t):
        # Calculate turn 
        self._target = tar
        #self._turning_angle = None
        #self._turning_angle_computed = False
        self.target_set = True
        self._target_set_time = t

    '''
        Change only the height (z) of the target
    '''
    def set_target_z(self, z, t):
        self._target[2] = z
        self._target_set_time = t

    '''
        Recover from lost tracking
    '''
    def __recover(self):
        pass

    def generate_control_signal(self, observation):
        return self.__generate_control_signal_dist(observation)
    
    def __set_within_interval(self):
        self._within_interval = False
        self._interval_timer.cancel()

    # observation is the current observation, a 12-list [x, y, z, 9 elements of the rotation matrix ...]
    # Return a 3-tuple (x_speed, y_speed, z_speed)
    def __generate_control_signal_dist(self, observation):
        ## If the object is out of tracking range, use the last valid observation
        track_lost = False
        if all([pos == 0 for pos in observation[:3]]) and self._last_tracked_observation is not None:
            observation = self._last_tracked_observation
            track_lost = True
        else:
            self._last_tracked_observation = observation

        curr_time = time.time()
        delta = -1
        then = np.array(self.state[:3], dtype = np.float64)
        then_yaw_dir = np.array(self.state[3:], dtype = np.float64)
        # print('state is %s' % str(self.state[3:]))  
        self.state[0:3] = observation[0:3]
        R = vec_to_mat(observation[3:])
        yaw_dir = R.dot(np.transpose(np.array([self._local_heading])))
        self.state[3:] = yaw_dir.flatten().tolist()
        ## Check if there is a recorded timestamp
        if self._last_timestamp is None:
            self._last_timestamp = curr_time
            return (None, None, None, None, None)
        else:
            delta = curr_time - self._last_timestamp
        ''' This IS actually distance '''
        signal_xyz = np.zeros((3, 1))
        signal_yaw = 0
        signal_xyz_speed = 0
        ## get speed
        now = np.array(observation[0:3], dtype = np.float64)
        speed = (now - then) / delta
        self.diff_state[0:3] = speed.tolist()
        ## Yaw speed (by comparing forward vector)
        ## TODO: Note this is only the absolute value, missing the direction (cw/ccw)
        # print(then_yaw_dir.dot(yaw_dir.flatten()))
        vel_yaw = abs(np.arccos(then_yaw_dir.dot(yaw_dir.flatten())) / np.pi * 180.0) / delta
        self.diff_state[3] = vel_yaw
        if not self._within_interval:
            try:
                speed_mag = np.linalg.norm(speed)
                #print('Speed are {0} {1}'.format(speed_mag, vel_yaw))
                print('Speed are {0}'.format(str(speed)))
                if speed_mag < self._still_speed_cap and vel_yaw < self._still_yaw_speed_cap: 
                    ## Speed below the cap indicates that the camera has finished flying one unit (large or small)
                    error = np.array(self._target[0:3], dtype=np.float64) - now
                    error_mag = np.linalg.norm(error)
                    ## TODO: try binary search instead of fixed step
                    print('Error is {0}'.format(error_mag))
                    if error_mag > self._dist_step_L:
                        ## in this case, the error is still quite large
                        signal_xyz = np.transpose(np.array([error / error_mag * self._dist_step_L]))
                        signal_xyz_speed = self._speed_L
                    elif error_mag > self._dist_step_S:
                        signal_xyz = np.transpose(np.array([error / error_mag * self._dist_step_S]))
                        signal_xyz_speed = self._speed_S
                    else:
                        tar_dir_world = np.array([[self._target[3]], [self._target[4]], [self._target[5]]], dtype=np.float64)
                        tar_dir = inv(R).dot(tar_dir_world) # in the local frame
                        tar_dir[2,0] = 0
                        tar_dir = tar_dir / np.linalg.norm(tar_dir)
                        sign = 1
                        if tar_dir[0, 0] < 0:
                            sign = -1
                        turning_angle = np.arccos(self._local_heading.dot(tar_dir.flatten())) * sign / np.pi * 180.0
                        if abs(turning_angle) > self._yaw_threshold:
                            ## Note negative input has to be reinterpreted as turning counter clockwise
                            signal_yaw = turning_angle
                else:
                    return (None, None, None, None, None)

                signal_trans = inv(R.dot(self._align_mat))
                local_signal = signal_trans.dot(signal_xyz) 
                self.recent_control_sig = [local_signal[0, 0], local_signal[1, 0], local_signal[2, 0], signal_yaw]
                self._within_interval = True
                self._interval_timer = threading.Timer(self._minimum_signal_interval, self.__set_within_interval)
                self._interval_timer.start()
                return (local_signal[0, 0], local_signal[1, 0], local_signal[2, 0], signal_xyz_speed, signal_yaw)
            except np.linalg.linalg.LinAlgError:
                return (None, None, None, None, None)
        else:
            return (None, None, None, None, None)