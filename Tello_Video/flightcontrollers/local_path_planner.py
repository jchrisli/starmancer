"""
Loosely based on Virtual Force Field (THE VECTOR FIELD HISTOGRAM -
FAST OBSTACLE AVOIDANCE FOR MOBILE ROBOTS)
"""

import numpy as np
from geometryUtils import point_to_line_dist, tangent_line_circle

class LocalPathPlanner(object):
    def __init__(self, vois2d):
        """Initialize the local path planner
        
        Arguments:
            object {[type]} -- [description]
            vois2d {array of 3 element numpy array} -- a list of voi (projected to the x-y plane) information, in the form of (cx, cy, r)
        """
        self._active_radius = 250
        self._search_angle = 90 / 180.0 * np.pi
        self._vois = vois2d

    def __intersect_2d_voi(self, pos, target):
        straight_dir = target - pos
        straight_dir = straight_dir / np.linalg.norm(straight_dir)
        for voi in self._vois:
            voir = voi[2]
            dist = point_to_line_dist(pos[:2], target[:2], voi[:2])
            if dist < voir:
                ## current move direction will intersect with this voi, get the safe boundary
                tangents = tangent_line_circle(pos[:2], voi[:2], voir)
                ## For now we only consider one obstacle
                return tangents
        return None


    def alter_velocity(self, pos, target, vel_mag):
        #TODO: if intersect with voi, get the two tangents and take the one that is closer to the 
        # direction of the target. Use this tangent as the boundry of the search area. 
        # Set new velocity to the center of the search area
