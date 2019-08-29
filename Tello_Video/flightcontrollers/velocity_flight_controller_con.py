from pid import PidController 
from velocity_flight_controller import DronePose
import math
import time
import numpy as np
from numpy.linalg import inv
from utils.geometryUtils import vec_to_mat
from colorama import Fore

class VelocityFlightControllerCon(object):

    Horizontal_Output_Range = (-0.4, 0.4)
    Vertical_Output_Range = (-0.4, 0.4)
    Yaw_Output_Range = (-0.6, 0.6)

    def __init__(self, local_planner):
        self._x_controller = PidController(False, 0.6 / 1000, 0.06 / 1000, 0)
        self._y_controller = PidController(False, 0.6 / 1000, 0.06 / 1000, 0)
        self._z_controller = PidController(False, 0.6 / 1000, 0.06 / 1000, 0)
        self._yaw_controller = PidController(True, 0.6, 0.00, 0.0)

        self._target_pose = None
        self._prev_target_pose = None
        self._prev_pose_time = 0
        self._target_vel = None # 4 element array
        ## Should arrive at the target before this time 
        self._target_time = None
        self._target_time_duration = None
        self._target_timeout_behavior = ''
        # self._target_ind = 0

        self._close_enough_xyz = 0.10 * 1000 
        self._close_enough_yaw = 0.10
        # local drone params
        self._local_heading = np.array([0, 1, 0])
        ## Note there is no need to align with this drone model, as the x, y, z axes match exactly with roll, pitch, thrust
        #self._align_mat = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        self._stabilize_time = 8.0

        self._curr_vel = np.array([0, 0, 0])
        self._last_odom_pose = None
        self._last_odom_time = 0

        # save the goals locally
        self._goals = None
        self._goals_ind = 0

        # local path planner
        self._local_planner = local_planner

    def __dirvec_to_angle(self, dirvec):
        vecsin = dirvec[1] / math.sqrt(dirvec[0] ** 2 + dirvec[1] ** 2)
        yaw = math.asin(vecsin)
        if dirvec[0] < 0:
            if dirvec[1] >= 0:
                yaw = math.pi - yaw
            else:
                yaw = -math.pi - yaw

        return yaw
    
    @staticmethod
    def __clamp(v, lower, upper):
        return min(upper, max(lower, v))

    def on_goals(self, goals):
        self._goals = goals
        # The first subgoal is the previous target pose
        if len(self._goals) > 0:
            target_pose = self._goals[0]['params'][-6:]
            yaw = self.__dirvec_to_angle(target_pose[3:])
            self._target_pose = DronePose(target_pose[0], target_pose[1], target_pose[2], yaw)
            self.__get_subgoal(1)

    def __get_subgoal(self, ind):
        if self._goals is not None and ind < len(self._goals):
            # self._goals_ind += 1
            self._goals_ind = ind
            n = self._goals[self._goals_ind]
            target_pos = n['params'][-6:]
            expected_time = n['exp_time']
            timeout_b = n['timeout_b']
            self._prev_pose_time = time.time()
            self._prev_target_pose = self._target_pose
            self.set_target(target_pos, expected_time, timeout_b)
            return True
        else:
            return False

    def get_next_subgoal(self):
        self.__get_subgoal(self._goals_ind + 1)

    def set_target(self, tarpos, tartime, timeoutb):
        if tarpos is not None:
            # first turn direction vector into yaw angle
            taryaw = self.__dirvec_to_angle(tarpos[3:])
            self._target_pose = DronePose(tarpos[0], tarpos[1], tarpos[2], taryaw)
            diff = np.array(self._prev_target_pose.vec_to(self._target_pose))
            self._target_vel = diff / tartime
            self._target_time = tartime + time.time()
            self._target_time_duration = tartime
            self._target_timeout_behavior = timeoutb
        else:
            self._target_pose = None

    ## Called upon positional tracking readings
    ## odom is a 6-tuple [x, y, z, dirvec_x, dirvec_y, dirvec_z] 
    def odom_callback(self, odom, debug = False):
        """Called upon odometry readings to generate control signal. Note the return signal is (roll -- right, pitch -- forward, thrust --up, yaw -- clockwise)
        
        Arguments:
            odom {[6 tuple]} -- [x, y, z, dirvec_x, dirvec_y, dirvec_z]
        
        Keyword Arguments:
            debug {bool} -- [in debug mode or not] (default: {False})
        """
        if self._target_pose is None:
            return 0, [0, 0, 0, 0] 
        # Process odometry data
        R = vec_to_mat(odom[3:])
        yaw_dir = R.dot(np.transpose(np.array([self._local_heading])))
        curr_yaw = self.__dirvec_to_angle(yaw_dir.flatten().tolist())
        curr_pose = DronePose(odom[0], odom[1], odom[2], curr_yaw)
        # Return integer state: 0 -- continue 1 -- abort
        retval = 0
        retvec = None
        now = time.time()
        # if now > self._target_time:
        if len(self._goals) == 0:
            # if the controller is in the stationary state
            retval = 1
        if self._target_pose.close_enough(curr_pose, self._close_enough_xyz, self._close_enough_yaw):
            # print('Close enough to the target.')
            # retval = 1
            self.__get_subgoal(self._goals_ind + 1)
        elif now > self._target_time + self._stabilize_time:
            if self._target_timeout_behavior == 'abort':
                retval = 1
            else:
                self.__get_subgoal(self._goals_ind + 1)
        if retval == 0 or debug:
            if self._last_odom_pose is None:
                self._last_odom_pose = curr_pose
            elapsed_time = now - self._prev_pose_time
            # interim_target = self._prev_target_pose.move(self._target_vel, elapsed_time)
            interim_target = self._prev_target_pose.move_timed(self._target_pose, self._target_time_duration, elapsed_time)
            #print('Elapsed time %s total time %s' % (str(elapsed_time), str(self._target_time_duration)))
            self._x_controller.set_target(interim_target.x)
            self._y_controller.set_target(interim_target.y)
            self._z_controller.set_target(interim_target.z)
            self._yaw_controller.set_target(interim_target.yaw)
            # print('distance to interim target is %s' % str(self._last_odom_pose.vec_to(interim_target)))
            
            # first data hack
            dt = now - self._last_odom_time if self._last_odom_time != 0 else 0.1
            ubar_x = self._x_controller.calc(self._last_odom_pose.x, dt, 0.0)
            ubar_y = self._y_controller.calc(self._last_odom_pose.y, dt, 0.0)
            ubar_z = self._z_controller.calc(self._last_odom_pose.z, dt, 0.0)
            ubar_yaw = self._yaw_controller.calc(self._last_odom_pose.yaw, dt, 0.0)
            #print('First order error now is %s' % (str([self._target_pose.x - curr_pose.x, self._target_pose.y - curr_pose.y, self._target_pose.z - curr_pose.z, self._target_pose.yaw - curr_pose.yaw])))

            motion_signal = np.array([ubar_x, ubar_y, ubar_z])
            ## Do not use local path planning for now
            motion_signal = self._local_planner.alter_velocity(self._last_odom_pose.to_np_array(), interim_target.to_np_array(), motion_signal.flatten())
            motion_signal = np.reshape(motion_signal, (3, 1))
            # Rotate x, y velocity back to the drone's frame
            #signal_trans = inv(R.dot(self._align_mat))
            signal_trans = inv(R)
            # local_signal = signal_trans.dot(signal_xyz)
            local_signal = signal_trans.dot(motion_signal)
            retvec = local_signal.flatten().tolist() + [ubar_yaw]
            # print('Raw control signal is %s' % str(retvec))
            ## Clamp the output 
            retvec = [VelocityFlightControllerCon.__clamp(retvec[0], VelocityFlightControllerCon.Horizontal_Output_Range[0], VelocityFlightControllerCon.Horizontal_Output_Range[1]), \
                    VelocityFlightControllerCon.__clamp(retvec[1], VelocityFlightControllerCon.Horizontal_Output_Range[0], VelocityFlightControllerCon.Horizontal_Output_Range[1]), \
                    VelocityFlightControllerCon.__clamp(retvec[2], VelocityFlightControllerCon.Vertical_Output_Range[0], VelocityFlightControllerCon.Vertical_Output_Range[1]), \
                    VelocityFlightControllerCon.__clamp(retvec[3], VelocityFlightControllerCon.Yaw_Output_Range[0], VelocityFlightControllerCon.Yaw_Output_Range[1])]
        self._last_odom_pose = curr_pose
        self._last_odom_time = now
        return retval, retvec 


