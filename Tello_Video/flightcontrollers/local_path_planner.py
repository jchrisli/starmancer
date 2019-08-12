"""
Loosely based on Virtual Force Field (THE VECTOR FIELD HISTOGRAM -
FAST OBSTACLE AVOIDANCE FOR MOBILE ROBOTS)
"""

import numpy as np
from utils.geometryUtils import point_to_line_dist, tangent_line_circle, line_seg_intersect_circle

class LocalPathPlanner(object):
    def __init__(self, voi_manager):
        """Initialize the local path planner
        
        Arguments:
            voi_manager {object} -- Voi manager
        """
        self._active_radius = 1000
        self._search_angle = 90 / 180.0 * np.pi
        self._voi_manager = voi_manager

    def __intersect_2d_voi(self, pos, target):
        vois = self._voi_manager.get2dvois()
        for voi in vois:
            voir = voi[2]
            #dist = point_to_line_dist(pos, target, voi[:2])
            intersect = line_seg_intersect_circle(voi[:2], voir, pos, target)
            in_active = np.linalg.norm(voi[:2] - pos) - voir < self._active_radius
            if intersect and in_active:
                ## current move direction will intersect with this voi, get the safe boundary
                tangents = tangent_line_circle(pos, voi[:2], voir)
                ## For now we only consider one obstacle
                return tangents, np.linalg.norm(voi[:2] - pos) - voir
        return None, None

    def alter_velocity(self, pos, target, raw_vel):
        straight_dir = target[:2] - pos[:2]
        straight_dir = straight_dir / np.linalg.norm(straight_dir)
        tangents, dist = self.__intersect_2d_voi(pos[:2], target[:2])
        if tangents is not None:
            # At least one obstacle within the active range
            # Get the tangent direction that is closer to the target direction
            boundary = tangents[1] if \
                        np.arccos(straight_dir.dot(tangents[0])) > np.arccos(straight_dir.dot(tangents[1])) \
                        else tangents[0]
            # Determine which side the search area lies
            turn = np.cross(straight_dir, boundary)
            turn_angle = self._search_angle
            if turn < 0:
                turn_angle *= -1
            s_theta = np.sin(turn_angle)
            c_theta = np.cos(turn_angle)
            rot_m = np.array([[c_theta, s_theta], [s_theta, c_theta]])
            heading = rot_m.dot(np.reshape(boundary, (2, 1))).flatten()
            # Project the pid x-y velocity to heading
            # proj_vel = raw_vel[:2].dot(heading) * heading
            xy_vel_mag = (1 - dist / self._active_radius) * 0.6 
            xy_vel_mag = min(xy_vel_mag, 0.6)
            proj_vel = (xy_vel_mag / np.linalg.norm(heading)) * heading
            print('Raw velocity %s altered velocity %s' % (str(proj_vel), str(raw_vel[:2])))
            return np.append(proj_vel, raw_vel[2:])
        return raw_vel