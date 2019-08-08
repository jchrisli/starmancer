'''
    Generate a list of subgoals given the 
'''

import numpy as np
import functools
import math
import time
from pyquaternion import Quaternion

class ActionPlanner():
    def __init__(self, lck, voim):
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
        ## Anything above this height should be safe
        self._SAFE_Z = 1800
        self._CAMERA_AXIS_TILT = 12 / 180.0 * math.pi

        self._VEL = 300
        self._VEL_OC = 200

        self._voi_manager = voim
        self._plan_subscriber = None
    
    def __reset_subgoals(self):
        self._subgoals = []
        self._post_subgoals = []
        self._ind_goal_to_exec = 0
        self._post_subgoal_func = None

    def __set_subgoals(self, subgoals):
        self._subgoals = subgoals
        #self._post_subgoals = post_subgoals
        self._ind_goal_to_exec = 0
        self._post_subgoal_func = None
        ## Publish to subscribers
        if self._plan_subscriber is not None:
            goals_l = map(lambda g: g[0], self._subgoals)
            self._plan_subscriber.on_goals(goals_l)

    def add_plan_subscriber(self, subscriber):
        self._plan_subscriber = subscriber
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

    def __generate_along_line_subgoal(self, target, vdir, etime, timeout_behavior = 'abort'):
        return { #'goal': 'line', \
                'params': target.tolist() + vdir.tolist(), \
                #'complete_crit': lambda x: np.norm(np.array(x) - target) < self._line_task_min_dist}
                #'complete_crit': lambda x: x < self._line_task_min_dist,
                'exp_time': etime, \
                'timeout_b': timeout_behavior \
                # 'speed': self._VEL
                }
                #'complete_crit': self.__line_subgoal_complete_crit}

    def __update_current_voi(self, voi):
        print('Update current voi %s' % str(voi))
        self._current_orbit_voi = voi
        self._current_orbit_voi_id = voi['id']

    def __set_state(self, state):
        self._current_state = state

    def __post_dock_func(self, voi):
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
        end_target = voi_pos + view_dist * (-vdir)
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
            ## Add one subgoal that forces the camera to fly to the specified position
            g_final = self.__generate_along_line_subgoal(end_target, vdir)
            self._subgoals.append(g_final)
            self._post_subgoals.append(lambda : True)
            print('Voi subgoals are %s' % str(self._subgoals))
        except np.linalg.LinAlgError:
            print('LinAlg error. Cannot generate subgoals for action planning.')
        finally:
            self._goal_lock.release()

    """
        If there is any potential of contact just raise the camera up and come down, thus 'on stilts'
        Otherwise go straight
    """
    def generate_subgoals_voi_onstilts(self, position, curdir, voi, vdir, lookatpoint = None):
        sub = []
        # now = time.time()
        # post_sub = []
        init_position = np.array(position)
        view_dist = voi['view_dist']
        voi_pos = voi['position3d']
        vdir = np.array(vdir)
        vdir = vdir / np.linalg.norm(vdir)
        voi_radius = voi['size3d']
        curdir = np.array(curdir)
        # end_target = voi_pos + view_dist * (-vdir)
        if lookatpoint is None:
            end_target = voi_pos + view_dist * (-vdir) + np.array([0, 0, (view_dist - voi_radius) * math.tan(self._CAMERA_AXIS_TILT)])
        else:
            lookatpoint = np.array(lookatpoint)
            end_target = lookatpoint + (view_dist - voi_radius) * (-vdir) + np.array([0, 0, (view_dist - voi_radius) * math.tan(self._CAMERA_AXIS_TILT)])
        # prev_position = init_position
        ## Instead of turning to the target voi, turn to the average of begin and end look direction
        # look_dir = voi_pos - init_position
        will_intersect = self._voi_manager.test_path_against_all_voi(init_position, end_target)
        if will_intersect:
            look_dir = voi_pos - init_position
        else:
            look_dir = (vdir + curdir) / 2
            look_dir = end_target - init_position
        ## Set z to zero
        look_dir[2] = 0
        look_dir = look_dir / np.linalg.norm(look_dir)
        #g_turn = self.__generate_along_line_subgoal(init_position, look_dir)
        #sub.append((g_turn, lambda : True))
        # Set the starting point to be the first subgoal, mark it with a negative time
        first = self.__generate_along_line_subgoal(init_position, curdir, -1)
        sub.append((first, lambda: True))
        # Get a straight path, and check if it intersect with any of the vois
        #if will_intersect:
        #    ## so the path intersect with at least one of the vois, raise the camera to the safe height
        #    up_target = init_position + np.array([0, 0, self._SAFE_Z - init_position[2]])
        #    ng_time += (self._SAFE_Z - init_position[2]) / self._VEL
        #    g_up = self.__generate_along_line_subgoal(up_target, look_dir, ng_time)
        #    # prev_position = up_target
        #    sub.append((g_up, lambda : True))
        #    go_target = np.array([0, 0, self._SAFE_Z - end_target[2]]) + end_target
        #    ng_time +=  np.linalg.norm(go_target - up_target) / self._VEL
        #    g_go = self.__generate_along_line_subgoal(go_target, look_dir, ng_time)
        #    sub.append((g_go, lambda : True))
        #    ng_time += np.linalg.norm(end_target - go_target) / self._VEL
        #    go_down = self.__generate_along_line_subgoal(end_target, vdir, ng_time) #    sub.append((go_down, lambda : True))
        #else:
        ## Since the path does not intersect with any of the vois, go ahead
        ng_time = np.linalg.norm(end_target - init_position) / self._VEL
        go_straight = self.__generate_along_line_subgoal(end_target, vdir, ng_time)
        sub.append((go_straight, lambda : True))
        self.__set_subgoals(sub)
        print(self._subgoals)

    '''
        Generate a subgoal for turning to look at a point while staying place
    '''
    def generate_subgoals_look(self, position, cdir, look_pos):
        sub = []
        position = np.array(position)
        look_dir = np.array(look_pos) - position
        curr_dir = np.array(cdir)
        first = self.__generate_along_line_subgoal(position, curr_dir, -1)
        sub.append((first, lambda: True))
        look_dir[2] = 0
        look_dir = look_dir / np.linalg.norm(look_dir)
        g_look = self.__generate_along_line_subgoal(position, look_dir, 2.0) # ideally we should calculate this, but use 0.5 for now
        # with self._goal_lock:
        sub.append((g_look, lambda : True))
        self.__set_subgoals(sub)
        print(self._subgoals)

    def generate_subgoals_manual_orbit(self, position, cdir, voi, lr):
        sub = []
        position = np.array(position)
        curr_dir = np.array(cdir)
        first = self.__generate_along_line_subgoal(position, curr_dir, -1)
        sub.append((first, lambda: True))
        voi_center = voi['position3d']
        offset = position - voi_center
        orbit_center = np.array([voi_center[0], voi_center[1], position[2]])
        # Rotate the center from center to the drone to generate other waypoints
        if lr == 'l':
            d = -1
        else:
            d = 1
        rqs = [Quaternion(axis = [0, 0, 1], angle = d * i * math.pi / 4) for i in range(1, 5)]
        waypoints_offsets = map(lambda rq: rq.rotate(offset), rqs)
        waypoints = map(lambda offset: offset + voi_center, waypoints_offsets)
        for wp in waypoints:
            wp_dir = orbit_center - wp
            r = np.linalg.norm(wp_dir)
            t = math.sqrt(2) * r / self._VEL_OC
            wp_dir = wp_dir / r
            wp_goal = self.__generate_along_line_subgoal(wp, wp_dir, t, 'continue')
            sub.append((wp_goal, lambda: True))
        self.__set_subgoals(sub)
        print(self._subgoals)

    def generate_subgoals_manual_zoom(self, position, cdir, voi, io):
        sub = []
        position = np.array(position)
        curr_dir = np.array(cdir)
        first = self.__generate_along_line_subgoal(position, curr_dir, -1)
        sub.append((first, lambda: True))
        horizontal = np.append(voi['position3d'][:2] - position[:2], [0], axis=0)
        horizontal_dir = horizontal / np.linalg.norm(horizontal)
        horizontal_dist = np.linalg.norm(horizontal)
        if io == 'i':
            minimum_view_dist = max(voi['view_dist'] / 1.5, voi['size3d'] + 200)
            # print('horizontal dist, view dist, size3d: %s %s %s' % (horizontal_dist, voi['view_dist'], voi['size3d']))
            if horizontal_dist > minimum_view_dist:
                zoomin_pos = voi['position3d'] -  minimum_view_dist * horizontal_dir
                t = (horizontal_dist - minimum_view_dist) / self._VEL_OC
                zoomin_target = self.__generate_along_line_subgoal(zoomin_pos, horizontal_dir, t)
                sub.append((zoomin_target, lambda: True))
        elif io == 'o':
            maximum_view_dist = 2 * voi['view_dist']
            if maximum_view_dist > horizontal_dist:
                zoomout_pos =  voi['position3d'] - maximum_view_dist * horizontal_dir
                t = (maximum_view_dist - horizontal_dist) / self._VEL_OC
                zoomout_target = self.__generate_along_line_subgoal(zoomout_pos, horizontal_dir, t)
                sub.append((zoomout_target, lambda: True))
        self.__set_subgoals(sub)
        print(self._subgoals)
    
    '''
        Simply go to a position and turn to a direction
    '''
    def generate_subgoals_position(self, cpos, cdir, vposition, vdir):
        sub = []
        position = np.array(cpos)
        curr_dir = np.array(cdir)
        first = self.__generate_along_line_subgoal(position, curr_dir, -1)
        sub.append((first, lambda: True))
        vposition = np.array(vposition)
        vdir = np.array(vdir)
        t = np.linalg.norm(vposition - position) / self._VEL
        task = self.__generate_along_line_subgoal(vposition, vdir, t)
        sub.append((task, lambda:True))

        self.__set_subgoals(sub)
        print(self._subgoals)

    def generate_subgoals_appraoch(self, cur_position, cur_dir, voi):
        self.generate_subgoals_voi_onstilts(cur_position, cur_dir, voi, cur_dir)

    def abort_subgoals(self):
        self.__set_subgoals([])

    def next(self):
        if self._post_subgoal_func is not None:
            self._post_subgoal_func()
            self._post_subgoal_func = None
        #self._ind_goal_to_exec = self._ind_goal_to_exec + 1
        #print('Index of next goal %d' % self._ind_goal_to_exec)
        if self._ind_goal_to_exec == len(self._subgoals):
            return None
        else:
            (subgoal, post_subgoal) = self._subgoals[self._ind_goal_to_exec]
            self._post_subgoal_func = post_subgoal 
            self._ind_goal_to_exec = self._ind_goal_to_exec + 1
            return subgoal

    def __iter__(self):
        return self


