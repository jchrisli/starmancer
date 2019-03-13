'''
    Get an volume of interest based on two regions selected on the camera views
    Camera model part, see Hartley book Multi-view Geometry in Computer Vision, p161
'''

import numpy as np
import numpy.linalg

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
    def __init__(self):
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
        self._roiFpv = None
        self._roiTop = None

    '''
        Concatenate the two np array
    '''
    def update_fpv_ext(self, rot, trans):
        #self._extMatFpv = np.concatenate((rot, trans), axis=1)
        rot = np.array(rot).reshape(3, 3)
        rot = self._extViconToCamera.dot(rot)
        trans = np.array(trans).reshape(3, 1)
        self._extMatFpvR = numpy.linalg.inv(rot)
        self._extMatFpvT = - self._extMatFpvR.dot(trans)
        ## Update fpv camera calibration matrix
        self._matFpv = self._intMatFpv.dot(np.hstack((self._extMatFpvR, self._extMatFpvT)))

    def set_2d_roi_fpv(self, left, right, top, bottom):
        self._roiFpv = {'left': left, 'right': right, 'top': top, 'bottom': bottom}

    def set_2d_roi_top(self, left, right, top, bottom):
        self._roiTop = {'left': left, 'right': right, 'top': top, 'bottom': bottom}

    def get_roi_top_ground_intersection(self, left, right, top, bottom):
        roi = {'left': left, 'right': right, 'top': top, 'bottom': bottom}
        o, d = self._get_roi_ray(roi, self._matTop)
        o = o.flatten()
        d = d.flatten()
        n = np.array([0.0, 0.0, 1.0])
        p_n = np.array([0.0, 0.0, 0.0])
        t = n.dot(p_n - o) / (n.dot(d))
        intersection = o + t * d
        return intersection

    '''
        Return a ray in 3d, direction determined by the pixel clicked
        params class data structure: roi, camera matrix 3 * 4: p
        return (np.array(3, 1): origin, np.array(3, 1): direction)
        See http://answers.opencv.org/question/117354/back-projecting-a-2d-point-to-a-ray/?answer=123124#post-id-123124 for details
    '''
    def _get_roi_ray(self, roi, P):
        pixel = np.array([[(roi['left'] + roi['right']) / 2, (roi['top'] + roi['bottom']) / 2, 1]]).transpose()
        M_inv = numpy.linalg.inv(P[:,:3])
        p_4 = P[:, 3].reshape(3, 1)
        direction = M_inv.dot(pixel)
        origin = - M_inv.dot(p_4)
        return (origin, direction)

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
        print('s {0} f {1} d {2}'.format(s, f, dist))
        return r

    '''
        Return a volume of interest
        return (np.array(3): center_x, center_y, center_z, number: radius)
    '''
    def get_voi(self):
        ## First get the two rays extending from the camera to the center of the rois
        (rayFpvOrigin, rayFpvDir) = self._get_roi_ray(self._roiFpv, self._matFpv)
        (rayTopOrigin, rayTopDir) = self._get_roi_ray(self._roiTop, self._matTop)

        (voiCenter, alongFpvRay, alongTopRay, dist) = self._fuzzy_intersection(rayFpvOrigin, rayFpvDir, rayTopOrigin, rayTopDir)
        ## TODO: if the dist is too large, do not return voi
        ## FIXIT: there are various ways to calculate voi volume; use the simpliest one for now
        ## Consider the volume as a sphere, and set the radius of the sphere to be the largest of the roi width/height reprojected 
        ## back to 3d
        r1 = self._roi_to_3d(self._roiFpv, alongFpvRay, self._intMatFpv)
        r2 = self._roi_to_3d(self._roiTop, alongTopRay, self._intMatTop)
        #print('r1: {0}, r2: {1}'.format(r1, r2))
        voiRadius = max(r1, r2) / 2
        return (voiCenter, voiRadius)
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