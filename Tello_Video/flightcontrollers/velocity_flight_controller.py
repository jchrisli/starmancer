'''
Note the standard ROS units for length and time are meters and seconds. The controller parameters may need some change.
'''
from pid import PidController2 
import math
import time
import numpy as np
from numpy.linalg import inv
from utils.geometryUtils import vec_to_mat
from colorama import Fore

class DronePose():
    def __init__(self, x, y, z, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw 

    def close_enough(self, pose, xyz_thresh, yaw_thresh):
        return abs(self.x - pose.x) < xyz_thresh and \
            abs(self.y -pose.y) < xyz_thresh and \
            abs(self.z -pose.z) < xyz_thresh and \
            abs(self.yaw -pose.yaw) < yaw_thresh

class VelocityFlightController:

    Horizontal_Output_Range = (-0.4, 0.4)
    Vertical_Output_Range = (-0.4, 0.4)
    Yaw_Output_Range = (-0.4, 0.4)

    def __init__(self):
        self._x_controller = PidController2(False, 0.2 / 1000, 0.03 / 1000)
        self._y_controller = PidController2(False, 0.2 / 1000, 0.03 / 1000)
        self._z_controller = PidController2(False, 0.20 / 1000, 0.03 / 1000)
        self._yaw_controller = PidController2(True, 0.3, 0.0)

        self._target_pose = None
        self._target_vel = None # 4-tuple
        self._target_time = None
        # self._target_ind = 0

        self._close_enough_xyz = 0.05 * 1000 
        self._close_enough_yaw = 0.05
        # local drone params
        self._local_heading = np.array([0, 1, 0])
        ## Note there is no need to align with this drone model, as the x, y, z axes match exactly with roll, pitch, thrust
        #self._align_mat = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        self._stabilize_time = 5.0

        self._last_pose = None
        self._last_pose_time = 0

        # save the goals locally
        self._goals = None
        self._goals_ind = 0

        ### public
        # self.ntd = True

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
        self._goals_ind = 0

    def __get_next_subgoal(self):
        if self._goals is not None:
            if self._goals_ind + 1 < len(self._goals):
                self._goals_ind += 1
                n = self._goals[self._goals_ind]
                target_pos = n['params'][-6:]
                target_vel = None
                expected_time = n['exp_time']
                self.set_target(target_pos, target_vel, expected_time)


    def set_target(self, tarpos, tarvel, tartime):
        if tarpos is not None:
            # first turn direction vector into yaw angle
            taryaw = self.__dirvec_to_angle(tarpos[3:])
            self._target_pose = DronePose(tarpos[0], tarpos[1], tarpos[2], taryaw)
            self._target_vel = tarvel
            self._target_time = tartime
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
        # Return integer state: 0 -- continue 1 -- target reached 2 -- abort
        retval = 0
        retvec = None
        now = time.time()
        if now > self._target_time:
            if self._target_pose.close_enough(curr_pose, self._close_enough_xyz, self._close_enough_yaw):
                print('Close enough to the target.')
                # retval = 1
                self.__get_next_subgoal()
            elif now > self._target_time + self._stabilize_time:
                retval = 1
        if retval == 0 or debug:
            # Check if it is close enough to the target        
            if self._last_pose is None:
                self._last_pose = curr_pose
            dt = now - self._last_pose_time
            x_dot_actual = (curr_pose.x - self._last_pose.x) / dt
            y_dot_actual = (curr_pose.y - self._last_pose.y) / dt
            z_dot_actual = (curr_pose.z - self._last_pose.z) / dt
            yaw_dot_actual = (curr_pose.yaw - self._last_pose.yaw) / dt
            # Set target speed to be 0 for now
            ubar_x = self._x_controller.calc(self._target_pose.x, curr_pose.x, 0.0, x_dot_actual)
            ubar_y = self._y_controller.calc(self._target_pose.y, curr_pose.y, 0.0, y_dot_actual)
            ubar_z = self._z_controller.calc(self._target_pose.z, curr_pose.z, 0.0, z_dot_actual)
            ubar_yaw = self._yaw_controller.calc(self._target_pose.yaw, curr_pose.yaw, 0.0, yaw_dot_actual)
            print('First order error now is %s at %s' % (str([self._target_pose.x - curr_pose.x, self._target_pose.y - curr_pose.y, self._target_pose.z - curr_pose.z, self._target_pose.yaw - curr_pose.yaw]), time.time()))

            motion_signal = np.array([[ubar_x], [ubar_y], [ubar_z]])
            # Rotate x, y velocity back to the drone's frame
            #signal_trans = inv(R.dot(self._align_mat))
            signal_trans = inv(R)
            # local_signal = signal_trans.dot(signal_xyz)
            local_signal = signal_trans.dot(motion_signal)
            retvec = local_signal.flatten().tolist() + [ubar_yaw]
            #print('Raw control signal is %s' % str(retvec))
            ## Clamp the output 
            retvec = [VelocityFlightController.__clamp(retvec[0], VelocityFlightController.Horizontal_Output_Range[0], VelocityFlightController.Horizontal_Output_Range[1]), \
                    VelocityFlightController.__clamp(retvec[1], VelocityFlightController.Horizontal_Output_Range[0], VelocityFlightController.Horizontal_Output_Range[1]), \
                    VelocityFlightController.__clamp(retvec[2], VelocityFlightController.Vertical_Output_Range[0], VelocityFlightController.Vertical_Output_Range[1]), \
                    VelocityFlightController.__clamp(retvec[3], VelocityFlightController.Yaw_Output_Range[0], VelocityFlightController.Yaw_Output_Range[1])]
        self._last_pose = curr_pose
        self._last_pose_time = now
        return retval, retvec 


