'''
    Get an volume of interest based on two regions selected on the camera views
    Camera model part, see Hartley book Multi-view Geometry in Computer Vision, p161
'''

import numpy as np
import numpy.linalg
import math
import cv2

class VoiCalulator():
    '''
        pixel coords OpenCV
        ------------>(x)
        |
        |
        |
        |
        v
        (y)
    '''
    def __init__(self, camParams):
        '''
        self._intMatFpv = np.array([[885.5839394885504, 0.0, 488.2530454326725], \
                                    [0.0, 883.6668510262632, 362.2197785226864], \
                                    [0.0, 0.0, 1.0]])
        self._intMatTop = np.array([[788.9962872461164 / 4, 0.0, 627.0971100621931 / 4 ], \
                                    [0.0, 790.378055853906 / 4, 345.54398722442954 / 4], \
                                    [0.0, 0.0, 1.0]])
        ## This is someting 3 * 4 and constant
        self._extMatTopR = np.array([[0, 1, 0], \
                                    [1, 0, 0], \
                                    [0, 0, -1]])
        self._extMatTopT = np.array([[400.0], [0.0], [2600.0]])
        self._matTop = self._intMatTop.dot(np.hstack((self._extMatTopR, self._extMatTopT)))
        self._extViconToCamera = np.array([1, 0, 0, 0, 0, -1, 0, 1, 0]).reshape(3, 3, order='F')
        ## This is someting constantly changing
        self._extMatFpvR = None
        self._extMatFpvT = None
        self._matFpv = None
        ## 
        '''
        self._camParams = camParams
        self._matTop = self._camParams.get_mat_top()
        self._iMatTop = self._camParams.get_int_mat_top()
        self._iMatFpv = self._camParams.get_int_mat_fpv()
        self._distTop = self._camParams.get_top_dist()
        self._distFpv = self._camParams.get_fpv_dist()
        self._cam_pos = self._camParams.get_top_cam_position()
        self._cam_proj_g = np.array([self._cam_pos[0], self._cam_pos[1], self._cam_pos[2]])
        self._roiFpv = None
        self._roiTop = None

        ## Constants
        self._DEFAULT_RADIUS = 110
        self._DEFAULT_HALF_HEIGHT = self._DEFAULT_RADIUS
        # The target's height is between 0.5m and 1.75m
        self._TARGET_HEIGHT_HIGH = 1750.0
        self._TARGET_HEIGHT_LOW = 500.0

    def set_2d_roi_fpv(self, left, right, top, bottom, t):
        self._roiFpv = {'type': t, 'left': left, 'right': right, 'top': top, 'bottom': bottom}

    def set_2d_roi_top(self, left, right, top, bottom, angle, t):
        self._roiTop = {'type': t, 'angle': angle, 'left': left, 'right': right, 'top': top, 'bottom': bottom}

    def __get_visible_area(self, rayintersection):
        """Get the center and radius of the circle, which the fpv camera that can see the entire possible target range lies on 
        
        Arguments:
            rayintersection {numpy array} -- where the ray intersect with the ground
        """
        camz = self._cam_pos[2]
        t_h = self._TARGET_HEIGHT_HIGH / camz
        t_l = self._TARGET_HEIGHT_LOW / camz
        proj_dir = self._cam_proj_g - rayintersection
        proj_dir = proj_dir / np.linalg.norm(proj_dir)
        low_proj_g = (1 - t_l) * rayintersection + t_l * self._cam_proj_g
        high_proj_g = (1 - t_h) * rayintersection + t_h * self._cam_proj_g
        center_proj_g = (low_proj_g + high_proj_g) / 2.0
        ## get the vector that is perpendicular to the chord
        chord_normal_y = -proj_dir[0] / proj_dir[1] 
        chord_normal = np.array([1, chord_normal_y, 0])
        chord_normal = chord_normal / np.linalg.norm(chord_normal)
        chord_center_to_fpv = self._camParams.get_fpv_cam_position() - center_proj_g
        chord_center_to_fpv = chord_center_to_fpv / np.linalg.norm(chord_center_to_fpv)
        center_fpv_proj_chord = chord_normal * (chord_normal.dot(chord_center_to_fpv))
        half_chord = np.linalg.norm(high_proj_g - low_proj_g) / 2.0
        area_center =  center_proj_g + half_chord / math.tan(self._camParams.get_fov()) * center_fpv_proj_chord
        area_r = half_chord / math.sin(self._camParams.get_fov())
        return area_center, area_r


    def _get_ray_ground_intersection(self, rayori, raydir):
        o = rayori.flatten()
        d = raydir.flatten()
        n = np.array([0.0, 0.0, 1.0])
        p_n = np.array([0.0, 0.0, 0.0])
        t = n.dot(p_n - o) / (n.dot(d))
        intersection = o + t * d
        return intersection

    def get_roi_top_ground_intersection(self, left, right, top, bottom):
        roi = {'left': left, 'right': right, 'top': top, 'bottom': bottom}
        o, d = self._get_roi_ray(roi, self._matTop, self._iMatTop, self._distTop)
        ## Get the ray-ground intersection for three of the corners
        threecorners = [np.array([[left], [top], [1]]), np.array([[right], [top], [1]]), np.array([[left], [bottom], [1]])]
        threerays = map(lambda c: self._get_pixel_ray(c, self._matTop), threecorners)
        threeintersections = map(lambda ray: self._get_ray_ground_intersection(ray[0], ray[1]), threerays)
        xlen = numpy.linalg.norm(threeintersections[1] -  threeintersections[2])
        ylen = numpy.linalg.norm(threeintersections[0] -  threeintersections[1])
        groundintersection = self._get_ray_ground_intersection(o, d)
        # camorigin = self._camParams.get_ext_mat_top_inv()[:3, 3].flatten()
        # lookp = (groundintersection + camorigin) / 2.0
        fpv_area_c, fpv_area_r = self.__get_visible_area(groundintersection)
        print('visible area center %s radius %s' % (fpv_area_c, fpv_area_r))
        ## Check if the current camera position is in the area
        fpv_pos = self._camParams.get_fpv_cam_position()
        fpv_area_c[2] = fpv_pos[2]
        radius_vec = fpv_pos - fpv_area_c
        dist_to_c =  np.linalg.norm(radius_vec) 
        print('Camera dist to c is %s' % dist_to_c)
        if dist_to_c < fpv_area_r:
            ## too close, back up
            movep = fpv_area_c + fpv_area_r * radius_vec / dist_to_c
        else:
            movep = None
        ## Always look at the center of the possible camera area 
        return (movep, fpv_area_c, threeintersections[0], xlen, ylen)


    '''
        Return a ray in 3d, direction determined by the pixel clicked
        params class data structure: roi, camera matrix 3 * 4: p
        return (np.array(3, 1): origin, np.array(3, 1): direction)
        See http://answers.opencv.org/question/117354/back-projecting-a-2d-point-to-a-ray/?answer=123124#post-id-123124 for details
    '''
    def _get_pixel_ray(self, pix, P):
        M_inv = numpy.linalg.inv(P[:,:3])
        p_4 = P[:, 3].reshape(3, 1)
        direction = M_inv.dot(pix)
        origin = - M_inv.dot(p_4)
        return (origin, direction)

    def _get_roi_ray(self, roi, P, iMat, dist):
        pixel = np.array([[(roi['left'] + roi['right']) / 2, (roi['top'] + roi['bottom']) / 2]],  dtype = np.float32)
        print('orignal pix is %s' % pixel)
        # undistort points here
        pixel_undistorted = cv2.undistortPoints(pixel.reshape(-1, 1, 2), iMat, dist, None, iMat.copy())
        print('undistorted pix is %s' % pixel_undistorted)
        return self._get_pixel_ray(np.vstack((pixel_undistorted[0,:,:].transpose(), np.array([[1]], dtype=np.float32))), P)

    '''
        Find a line sgetment that gives the shortest distance between two ray
        Return the center of the segment, and the shortest distance
        Adapted from https://gist.github.com/rarora7777/7e08a3f39484e0cd984e9f34bd41b0be
        params p_i, d_i, both np.array(3, 1)
        return (np.array(3): center, number: distance)
    '''
    def _fuzzy_intersection(self, p1, d1, p2, d2):
        d1 = d1.flatten()
        d2 = d2.flatten()
        p1 = p1.flatten()
        p2 = p2.flatten()
        n = np.cross(d1, d2)
        n1 = np.cross(d1, n)
        n2 = np.cross(d2, n)

        t1 = (p2 - p1).dot(n2) / (d1.dot(n2))
        t2 = (p1 - p2).dot(n1) / (d2.dot(n1))

        c1 = p1 + t1 * d1
        c2 = p2 + t2 * d2
        c = (c1 + c2) / 2

        dist = numpy.linalg.norm(c1 - c2)

        return (c, t1, t2, dist)

    '''
        Reproject a 2d roi to a 3d shape, for now just get a circle back
        params class data structure: roi, np.array(3, 3): internal matrix
        return number: radius
    '''
    def _roi_to_3d(self, roi, dist, internal):
        w = roi['right'] - roi['left']
        h = roi['bottom'] - roi['top']
        s = max(w, h)
        f = (internal[0, 0] + internal[1, 1]) / 2
        r = s * dist / f
        # print('s {0} f {1} d {2}'.format(s, f, dist))
        return r

    '''
        Reproject a 2d roi back to 3d, choosing a method depending on the type of the 
        roi (line vs contour) 
        No contour implementation at this point
        Do not handle 'dot' roi type
    '''
    def _typed_roi_to_3d(self, roi, dist, internal_mat):
        w = roi['right'] - roi['left']
        h = roi['bottom'] - roi['top']
        diag = math.sqrt(w ** 2 + h ** 2)  / 1.414
        f = (internal_mat[0, 0] + internal_mat[1, 1]) / 2
        diameter = diag * dist / f
        return diameter 

    '''
        Get both height and radius from this view
    '''
    def _fpv_roi_to_3d(self, roi, center, dist, internal_mat, external_mat):
        W = roi['right'] - roi['left']
        f = (internal_mat[0, 0] + internal_mat[1, 1]) / 2
        H = roi['bottom'] - roi['top']
        r = (W / 2) * dist / f
        ## (x, y, z) is the center of the cylinder in the camera frame
        (x, y, z) = external_mat.dot(np.append(center, 1.0).reshape(4, 1)).flatten().tolist()
        ## h is the root a quadratic equation
        valsin = math.sin(12 / 180.0 * math.pi)
        valcos = math.cos(12 / 180.0 * math.pi)
        valcot = 1 / (valsin / valcos)
        valcsc = 1 / valsin
        h = (1/(2 * H)) * (4 * f * r + 4 * H * r * valcot + 4 * f * y * valcsc -  \
            4 * f * z * valcot * valcsc + \
            valcsc * valcsc * math.sqrt(-4 * H * valsin * valsin * \
            (- 4 * H * z * z + 8 * f * r * y * valcos + \
            4 * H * r * r * valcos * valcos + \
            8 * f * r * r * valcos * valsin) + (4 * f * z * valcos - \
            4 * f * y * valsin - \
            4 * H * r * valcos * valsin - \
            4 * f * r * valsin * valsin) ** 2))
        #scale = 1.2
        scale = 1.0
        return (r * scale, h / 2 * scale) 

    def _get_cylinder_3d_aabb(self, center, r, h):
        voffset = h
        top_corner_1 = center + np.array([0, r, voffset])
        top_corner_2 = center + np.array([r, 0, voffset])
        top_corner_3 = center + np.array([0, -r, voffset])
        top_corner_4 = center + np.array([-r, 0, voffset])
        
        bottom_corner_1 = center + np.array([0, r, -voffset])
        bottom_corner_2 = center + np.array([r, 0, -voffset])
        bottom_corner_3 = center + np.array([0, -r, -voffset])
        bottom_corner_4 = center + np.array([-r, 0, -voffset])

        return np.vstack((top_corner_1, top_corner_2, top_corner_3, top_corner_4,\
            bottom_corner_1, bottom_corner_2, bottom_corner_3, bottom_corner_4))

    def _get_2d_aabb(self, corners, P):
        corners_proj = P.dot(np.vstack((corners.T, np.ones((1, corners.shape[0])))))
        s = corners_proj.shape
        last_row_m = np.tile(corners_proj[-1,:], (s[0], 1))
        normalized = np.divide(corners_proj, last_row_m)
        xy_max = np.amax(normalized[:2,:], axis = 1)
        xy_min = np.amin(normalized[:2,:], axis = 1)
        # Return (left, right, top, bottom)
        return {'left': xy_min[0], 'right': xy_max[0], 'top': xy_min[1], 'bottom': xy_max[1]}
        
    def _get_coord_in_roi(self, p, roi):
        return ((p[0, 0] - roi['left']) / (roi['right'] - roi['left']), (p[1, 0] - roi['top']) / (roi['bottom'] - roi['top']))
        
    def _p_from_roi_coord(self, roi, roi_coord):
        roi_w = roi['right'] - roi['left']
        roi_h = roi['bottom'] - roi['top']
        return np.array([roi['left'] + roi_coord[0] * roi_w, roi['top'] + roi_coord[1] * roi_h, 1]).reshape(3, 1)

    '''
        Return a volume of interest
    '''
    def get_voi(self):
        matFpv = self._camParams.get_mat_fpv()
        intMatFpv = self._camParams.get_int_mat_fpv()
        extMatFpv = self._camParams.get_ext_mat_fpv()
        matTop = self._matTop
        # intMatTop = self._camParams.get_int_mat_top()
        ## First get the two rays extending from the camera to the center of the rois
        (rayFpvOrigin, rayFpvDir) = self._get_roi_ray(self._roiFpv, matFpv, self._iMatFpv, self._distFpv)
        (rayTopOrigin, rayTopDir) = self._get_roi_ray(self._roiTop, matTop, self._iMatTop, self._distTop)

        (voiCenter, alongFpvRay, alongTopRay, dist) = self._fuzzy_intersection(rayFpvOrigin, rayFpvDir, rayTopOrigin, rayTopDir)
        ## r_fpv is the height
        # (r_top, r_fpv) = self._fpv_roi_to_3d(self._roiFpv, voiCenter, alongFpvRay, intMatFpv, extMatFpv)

        ## calculate the voi parameters again based on the first result
        # first reproject the obtained voi center back to the top view
        # first_center = np.vstack((voiCenter.reshape(3, 1), np.array([[1]])))
        # print('Voi center is at %s' % str(voiCenter))

        (first_r_top, first_r_fpv) = self._fpv_roi_to_3d(self._roiFpv, voiCenter, alongFpvRay, intMatFpv, extMatFpv)
        # print('Voi radius is %s half height is %s' % (second_r_top, second_r_fpv))
        #reprojected_center_top = matTop.dot(first_center)
        #reprojected_center_top = reprojected_center_top / reprojected_center_top[2, 0]

        ## Adjust the center position in the top cam view based on where the projection of the 
        ## center lies in the axis-aligned bounding box for the first result
        # Get the aabb of the cylinder from the first result
        #bb_cornders = self._get_cylinder_3d_aabb(voiCenter, first_r_top, first_r_fpv)
        #print('The 3d bounding box is %s' % str(bb_cornders))
        # Get the 2d projection of the bb cornders
        #bb2d = self._get_2d_aabb(bb_cornders, matTop)
        #print('The 2d bounding box is %s' % str(bb2d))
        # Get where the center projection lies in the 2d aabb
        #roi_coord = self._get_coord_in_roi(reprojected_center_top, bb2d)
        #print('roi coord is %s %s' % roi_coord)
        # adj_top_center = self._p_from_roi_coord(self._roiTop, roi_coord)
        # adj_top_ray_o, adj_top_ray_d = self._get_pixel_ray(adj_top_center, self._matTop)
        # (second_center, second_along_fpv, second_along_top, second_dist) = self._fuzzy_intersection(rayFpvOrigin, rayFpvDir, adj_top_ray_o, adj_top_ray_d)

        #(second_r_top, second_r_fpv) = self._fpv_roi_to_3d(self._roiFpv, second_center, second_along_fpv, intMatFpv, extMatFpv)
        # r_fpv = self._roi_to_3d(self._roiFpv, alongFpvRay, intMatFpv)
        #print('r1: {0}, r2: {1}'.format(r1, r2))
        # voiRadius = max(r1, r2) / 2

        # Study I 
        #if voiCenter[0] > 0:
        #    voiCenter = voiCenter + np.array([-200 if voiCenter[1] < 0 else -100, 0, 0])

        return (voiCenter, first_r_top, first_r_fpv)
        # return (second_center, second_r_top, second_r_fpv)
        #return (voiCenter, voiRadius, rayFpvOrigin, rayFpvDir, rayTopOrigin, rayTopDir)

def draw_sphere(cx, cy, cz, r, ax):
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = r * np.cos(u)*np.sin(v) + cx
    y = r * np.sin(u)*np.sin(v) + cy
    z = r * np.cos(v) + cz
    ax.plot_wireframe(x, y, z, color="g")

def test_voi():
    ## FPV: (960 * 720), Topdown: 320 * 180
    fpvExtR = [0.0, -1.0, 0.0, 0.0, 0, 1.0, -1.0, 0.0, 0]
    fpvExtT = [0.0, -600.0, 1000.0]
    v = VoiCalulator()
    v.set_2d_roi_fpv(430, 530, 310, 410)
    v.set_2d_roi_top(144, 174, 74, 124)
    v.update_fpv_ext(fpvExtR, fpvExtT)
    #(center, radius) = v.get_voi()
    voiC, voiR, fpvO, fpvD, topO, topD = v.get_voi()
    #print(str(fpvO))
    print(voiR)
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    xDim = 1500.0
    yDim = 2500.0
    zDim = 2500.0
    f = plt.figure()
    ax = f.gca(projection='3d')
    arrowLength = 3000.0
    ## Plot the fpv line of sight
    ax.quiver(fpvO[0, 0], fpvO[1, 0], fpvO[2, 0], arrowLength * fpvD[0, 0], arrowLength * fpvD[1, 0], arrowLength * fpvD[2, 0], color='red')
    #ax.set_xlim(0,10);ax.set_ylim(0,10);ax.set_zlim(0,10)
    ax.quiver(topO[0, 0], topO[1, 0], topO[2, 0], arrowLength * topD[0, 0], arrowLength * topD[1, 0], arrowLength * topD[2, 0], color='blue')

    draw_sphere(voiC[0], voiC[1], voiC[2], voiR, ax)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    #ax.set_title(str(x[0])+","+str(x[1]))
    ax.set_xlim(-xDim,xDim);ax.set_ylim(-yDim,yDim);ax.set_zlim(0,zDim)

    #ax.view_init(elev=29, azim=-30)
    #fout = 'test_%s_01.png' % (str(x[0])+str(x[1]))
    #plt.savefig(fout)
    #ax.view_init(elev=29, azim=-60)
    #fout = 'test_%s_02.png' % (str(x[0])+str(x[1]))
    #plt.savefig(fout)
    plt.show()

if __name__ == '__main__':
    test_voi()