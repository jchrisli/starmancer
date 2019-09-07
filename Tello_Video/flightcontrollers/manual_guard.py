import numpy as np

class ManualGuard(object):
    def __init__(self, obstacles, boundary, projection_window):
        self._ol = obstacles
        self._b = boundary # minx, maxx, miny, maxy
        self._b_center = np.array([(boundary[0] + boundary[1]) / 2, (boundary[2] + boundary[3]) / 2], dtype = np.float32)
        self._proj_w = projection_window
        self._delay = 0.2
        self._command_to_speed = 1000.0

    def __proj_position(self, position, hdir, roll, pitch, thrust):
        pos_arr = np.array(position, dtype = np.float32)
        rv = roll * self._command_to_speed * (self._proj_w + self._delay)
        pv = pitch * self._command_to_speed * (self._proj_w + self._delay)
        tv = thrust * self._command_to_speed * (self._proj_w + self._delay)
        pitch_axis = np.array(hdir)
        thrust_axis = np.array([0, 0, 1], dtype=np.float32)
        roll_axis = np.cross(pitch_axis, thrust_axis)
        offset = rv * roll_axis + pv * pitch_axis + tv * thrust_axis
        print('Go by %s' % offset)
        return pos_arr + offset

    def __check_safety_obstacles(self, position):
        for ob in self._ol:
            if np.sqrt((position[0] - ob[0]) ** 2 + (position[1] - ob[1]) ** 2) < ob[2] and abs(position[2] - ob[3]) < ob[4]:
                return ob
        return None

    def __check_safety_boundary(self, position):
        return position[0] > self._b[0] and position[0] < self._b[1] and position[1] > self._b[2] and position[1] < self._b[3]

    def __check_future_position_for_boundary(self, future_pos, curr_pos):
        future_pos_2d = future_pos[:2]
        curr_pos_2d = curr_pos[:2]
        return np.linalg.norm(future_pos_2d - self._b_center) < np.linalg.norm(curr_pos_2d - self._b_center)

    def __check_future_position_for_obstacle(self, future_p, curr_p, ob):
        ob_center = np.array(ob[:2] + tuple([ob[3]]), dtype=np.float32)
        return np.linalg.norm(future_p - ob_center) > np.linalg.norm(curr_p - ob_center)

    def check_safety(self, position, hdir, roll, pitch, thrust):
        curr_pos_arr = np.array(position, dtype = np.float32)
        # safe_b = self.__check_safety_boundary(position)
        safe_b = True
        not_safe_o = self.__check_safety_obstacles(position)
        if not safe_b or not_safe_o is not None:
            #print('Likely not safe')
            future_p = self.__proj_position(position, hdir, roll, pitch, thrust)
            bs = safe_b or self.__check_future_position_for_boundary(future_p, curr_pos_arr)
            os = not_safe_o is None or self.__check_future_position_for_obstacle(future_p, curr_pos_arr, not_safe_o)  
            #print('Bs and os are %s %s' % (bs, os))
            #print('Safeb and safeo are %s %s' % (safe_b, not_safe_o is None))
            return bs and os
        else:
            return True
                
    
