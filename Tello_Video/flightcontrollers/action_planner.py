'''
    Generate a list of subgoals given the 
'''

import numpy as np
import functools

class ActionPlanner():
    def __init__(self, lck):
        self._goal_lock = lck
        self._current_state = 'flying' ## or 'orbiting'
        self._current_orbit_voi_id = -1 ## id of the current voi
        self._current_orbit_voi = None
        #self._subgoals = []
        #self._post_subgoals = []
        #self._ind_goal_to_exec = -1
        #self._post_subgoal_func = None
        self.__reset_subgoals()
        #self._voi_mana = voi_manager
        self._line_task_min_dist = 283
        self._arc_subgoal_completed_called = 0
    
    def __reset_subgoals(self):
        self._subgoals = []
        self._post_subgoals = []
        self._ind_goal_to_exec = 0
        self._post_subgoal_func = None
    '''
        Generate parameters for moving along an arc from x1 to x2, with center c and radius r
    '''
    def __along_arc(self, x1, x2, c, r):
        #x1 = np.array(x1)
        #x2 = np.array(x2)
        #c = np.array(c)
        mid = (x1 + x2) / 2
        middir = (mid - c)
        middir = middir / np.linalg.norm(middir)
        x3 = c + middir * r
        return x3
    
    def __intersect_with_voi(self, x1, x2, c, r):
        ## The line is vec_x1 + t * (vec_x2 - vec_x1)
        ## The point is vec_c
        ## From https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        n = x2 - x1
        n = n / np.linalg.norm(n)
        half_bottom_side = ((x1 - c).dot(n)) * n 
        dist = (x1 - c) - half_bottom_side
        dist = np.linalg.norm(dist)
        if dist <= r:
            return x1 - 2 * half_bottom_side
        else:
            return None

    def __arc_subgoal_complete_crit(self, error):
        self._arc_subgoal_completed_called += 1
        if self._arc_subgoal_completed_called > 1:
            self._arc_subgoal_completed_called = 0
            return True
        else:
            return False

    def __generate_along_arc_subgoal(self, cur_position, target, center, r, vdir):
        mid_point = self.__along_arc(cur_position, target, center, r)
        return {'goal': 'arc', \
            'params': mid_point.tolist() + target.tolist() + vdir.tolist(), \
            ## Always return true
            'complete_crit': self.__arc_subgoal_complete_crit}

    def __generate_along_line_subgoal(self, target, vdir):
        return {'goal': 'line', \
                'params': target.tolist() + vdir.tolist(), \
                #'complete_crit': lambda x: np.norm(np.array(x) - target) < self._line_task_min_dist}
                'complete_crit': lambda x: x < self._line_task_min_dist}

    def __update_current_voi(self, voi):
        print('Update current voi %s' % str(voi))
        self._current_orbit_voi = voi
        self._current_orbit_voi_id = voi['id']

    def __set_state(self, state):
        self._current_state = state

    def __post_dock_func(self, voi):
        print('Call post dock function')
        self.__set_state('orbiting')
        self.__update_current_voi(voi)

    '''
        Generate a (line) subgoal to approach a voi
    '''
    def __dock(self, cur_pos, voi):
        voi_view_dist = voi['view_dist']
        voi_pos = voi['position3d']
        vdir = voi_pos - cur_pos
        vdir = vdir / np.linalg.norm(vdir)
        target = voi_pos - voi_view_dist * vdir
        return (self.__generate_along_line_subgoal(target, vdir), target, functools.partial(self.__post_dock_func, voi=voi))

    def __orbit(self, cur_pos, voi_pos, voi_view_dist, vdir):
        target = voi_pos + voi_view_dist * (-vdir)
        return (self.__generate_along_arc_subgoal(cur_pos, target, voi_pos, voi_view_dist, vdir), target)

    '''
        Generate a list of subgoals to realize the goal of observing voi, from dir
    '''
    def generate_subgoals_voi(self, position, voi, vdir):
        self._goal_lock.acquire()
        self.__reset_subgoals()
        init_position = np.array(position)
        view_dist = voi['view_dist']
        voi_pos = voi['position3d']
        voi_id = voi['id']
        vdir = np.array(vdir)
        # end_target = voi_pos + view_dist * (-vdir)
        prev_position = init_position
        try:
            if self._current_state == 'orbiting' :
                current_voi_pos = self._current_orbit_voi['position3d']
                current_voi_view_dist = self._current_orbit_voi['view_dist']
                if self._current_orbit_voi_id == voi_id:
                    ## So still focusing on the safe voi, just changing direction
                    ## Move along an arc from the current position to target position
                    #target = voi_pos + view_dist * (-vdir)
                    # self._subgoals.append(self.__generate_along_arc_subgoal(init_position, end_target, voi_pos, view_dist, vdir))
                    g_same = self.__orbit(init_position, voi_pos, view_dist, vdir)[0]
                    self._subgoals.append(g_same)
                    self._post_subgoals.append(lambda : True) ## do nothing
                else:
                    ## So moving to another voi
                    ## Check if the direct path intersect with current voi
                    intersect = self.__intersect_with_voi(init_position, voi_pos, current_voi_pos, current_voi_view_dist)
                    if intersect is not None:
                        ## 'Undocking': move to a position that faces the voi
                        ## Generate an arc sub-goal
                        g1_target = intersect
                        g1_vdir = voi_pos - current_voi_pos
                        g1_vdir = g1_vdir / np.linalg.norm(g1_vdir)
                        g1 = self.__generate_along_arc_subgoal(init_position, g1_target, current_voi_pos, current_voi_view_dist, g1_vdir)
                        prev_position = g1_target
                        self._subgoals.append(g1)
                        ## Set the state to 'flying' after undocking
                        self._post_subgoals.append(lambda : self.__set_state('flying')) 
                    ## go straight 
                    #g2_vdir = voi_pos - position
                    #g2_vdir = g2_vdir / np.linalg.norm(g2_vdir)
                    #g2_target = voi - view_dist * g2_vdir
                    (g2, g2_target, g2_post) = self.__dock(position, voi)
                    prev_position = g2_target
                    # g2 = self.__generate_along_line_subgoal(g2_target, g2_vdir)
                    self._subgoals.append(g2)
                    self._post_subgoals.append(g2_post)
                    ## Now it's close to the target voi, start orbiting around it
                    # g3_target = end_target
                    # g3 = self.__generate_along_arc_subgoal(prev_position, g3_target, voi_pos, view_dist, vdir)
                    (g3, g3_target) = self.__orbit(prev_position, voi_pos, view_dist, vdir)
                    self._subgoals.append(g3)
                    self._post_subgoals.append(lambda : True)
            else:
                ## The state is in 'flying'
                ## Just dock and then orbit
                (g_dock, dock_target, post_g_dock) = self.__dock(position, voi)
                prev_position = dock_target
                self._subgoals.append(g_dock)
                self._post_subgoals.append(post_g_dock)
                (g_orbit, orbit_target) = self.__orbit(prev_position, voi_pos, view_dist, vdir)
                self._subgoals.append(g_orbit)
                self._post_subgoals.append(lambda : True)
            print('Voi subgoals are %s' % str(self._subgoals))
        except np.linalg.LinAlgError:
            print('LinAlg error. Cannot generate subgoals for action planning.')
        finally:
            self._goal_lock.release()

    '''
        Generate a subgoal for turning to look at a point while staying place
    '''
    def generate_subgoals_look(self, position, look_pos):
        position = np.array(position)
        look_dir = np.array(look_pos) - position
        look_dir = look_dir / np.linalg.norm(look_dir)
        g_look = self.__generate_along_line_subgoal(position, look_dir)
        with self._goal_lock:
            self.__reset_subgoals()
            self._subgoals.append(g_look)
            self._post_subgoals.append(lambda : True)
            print(self._subgoals)

    def next(self):
        if self._post_subgoal_func is not None:
            self._post_subgoal_func()
            self._post_subgoal_func = None
        #self._ind_goal_to_exec = self._ind_goal_to_exec + 1
        print('Index of next goal %d' % self._ind_goal_to_exec)
        if self._ind_goal_to_exec == len(self._subgoals):
            return None
        else:
            ret = self._ind_goal_to_exec
            self._post_subgoal_func = self._post_subgoals[self._ind_goal_to_exec]
            self._ind_goal_to_exec = self._ind_goal_to_exec + 1
            return self._subgoals[ret]

    def __iter__(self):
        return self


