'''
    We like to use the term 'situated interaction'
    So this is trying to make sense of the situation
'''
from utils.colorPrint import color_print 
from utils.geometryUtils import vec_to_mat
import numpy as np

class Actor():
    def __init__(self):
        self._position = None
        self._rotation = None

    def get_position(self):
        return self._position

    def get_rotation(self):
        return self._rotation

    def set_position(self, pos):
        self._position = np.array([[pos[0]], [pos[1]], [pos[2]]], dtype=np.float64)
        #print('Setting actor position to {0}'.format(str(self._position)))
    
    def set_rotation(self, rot):
        if(len(rot) == 9):
            self._rotation = vec_to_mat(rot)
        else:
            self._rotation = rot
        #print('Setting actor rotation to {0}'.format(str(self._rotation)))
        

class Situation():
    def __init__(self):
        self.actors = {
            'cam': Actor(),
            'human': Actor()
        }

    def get_actor(self, actor_name):
        if actor_name in self.actors:
            return self.actors[actor_name]
        else:
            color_print('Cannot find actor {0}'.format(actor_name), 'ERROR')

    def get_anothers_viewpoint(self, actor_name):
        '''
            Get the position and orientation of another actor, with some offset
            (so that the moving cam does not run into the actor)
        '''
        offset = 500
        #TODO: verify the direction of the right in the helmet's local coordinate systme
        # TODO: get right direction from the actor dictionary
        forward = np.array([[0], [1], [0]], dtype=np.float64)
        right = np.array([[1 * offset], [0], [0]], dtype=np.float64)
        actor = self.get_actor(actor_name)
        another_pos = actor.get_rotation().dot(right)
        another_pos = another_pos + actor.get_position()
        another_dir = actor.get_rotation().dot(forward)
        return [another_pos[0, 0], another_pos[1, 0], another_pos[2, 0], 
                another_dir[0, 0], another_dir[1, 0], another_dir[2, 0]]