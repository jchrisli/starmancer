
'''
    Run the 
'''

import argparse
from pynput import keyboard
from ViconConnection import ViconConnection
# import matplotlib.pyplot as plt
import cv2
import sys
import time
import json

class StudyRunner():
    '''
        task_id: 1 or 2
        participant_id: 1 to n
        training: True of False
    '''
    def __init__(self, task_id, participant_id, training, start_ind=0):
        self._tid = task_id
        self._pid = participant_id
        self._is_training = training
        '''
            possible states include 'started' 'ended' 'canceled' 'crashed'
        '''
        self._task_state = 'initialized'
        self._task_ind = 0
        self._VICON_PORT = 9004
        self._MODEL_NAME = 'FunkyTello'
        self._vconn = ViconConnection(self._VICON_PORT, self.on_vicon)
        rightnow = time.strftime('%H %M %S', time.localtime())
        self._vicon_file_name = '_'.join(['spatial', 't%s' % str(task_id), 'p%s' % str(participant_id), rightnow]) + '.csv'
        self._res_file_name = '_'.join(['result', 't%s' % str(task_id), 'p%s' % str(participant_id), rightnow]) + '.csv'
        self._vicon_file = None
        self._res_file = None

        self._curr_target_img = None

        ## Read the task config file
        if training:
            task_file_name = 'training.txt'
        else:
            task_file_name = '_'.join(['t%s' % task_id, 'p%s' % participant_id]) + '.txt'
        try:
            task_file = open(task_file_name, 'rt')
        except IOError:
            print('File %s cannot be openned. Quit.' % task_file)
            sys.exit(0)
        self._tasks = task_file.readlines()
        ## Split by space
        task_l = map(lambda tl : tl.split(' '), self._tasks)
        self._tasks = map(lambda t: (t[0], int(t[1])), task_l)
        self._task_ind = start_ind
        ## TODO: load the first task
        first_img_ind = self._tasks[0][1]
        self.__load_target_img(first_img_ind)
        ##TODO: possibly lazy read here
        task_file.close()

    def start(self):
        ## if not training, open the two files, and listen to Vicon packets
        if not self._is_training:
            self._vicon_file = open(self._vicon_file_name, 'wt')
            self._vicon_file.write('task, task_ind, target_name, timestamp, task_state, x, y, z')
            self._res_file = open(self._res_file_name, 'wt')
            self._res_file.write('task, task_ind, target_name, timestamp, task_event')
            self._vconn.start()
        ## Start keyboard listener
        with keyboard.Listener(on_press = self.on_key) as listener:
            listener.join()
        self.end()

    def __load_target_img(self, target):
        cv2.destroyAllWindows()
        self._curr_target_img = cv2.imread('images/t%s.jpg' % target)
        cv2.imshow('Current Target', self._curr_target_img)

    def on_vicon(self, data):
        # First convert bytes to string
        dataStr = data.decode("utf-8")
        # print('Data received is %s' % dataStr)
        # To JSON 
        dataJ = json.loads(dataStr)
        name = dataJ["Name"]
        translation = dataJ["Translation"]
        # rotation = dataJ["Rotation"]
        if name == self._MODEL_NAME and self._task_ind < len(self._tasks):
            vicon_line = ','.join([str(item) for item in [self._tasks[self._task_ind][0], self._task_ind, \
                self._tasks[self._task_ind][1], time.time(), self._task_state, \
                translation[0], translation[1], translation[2]]])
            self._vicon_file.write(vicon_line + '\n')

    def __write_to_res_file(self, event):
        ## About line terminator see, https://stackoverflow.com/a/6160082/1745547
        task_line = ','.join([str(item) for item in [self._tasks[self._task_ind][0], self._task_ind, self._tasks[self._task_ind][1], time.time(), event]])
        self._res_file.write(task_line + '\n')

    def __parse_task_str(self, task_str):
        keywords = task_str.split('_')
        technique = ''
        size = ''
        rep = ''
        if len(keywords) == 3:
            if keywords[0] == 'JS':
                technique = 'joystick'
            elif keywords[0] == 'SH':
                technique = 'starhopper'
            if keywords[1] == 'L':
                size = 'large'
            elif keywords[1] == 'S':
                size = 'small'
            rep = keywords[2]
            if any([len(s) == 0 for s in [technique, size, rep]]):
                raise Exception('Wrong task string content')
            else:
                return (technique, size, rep)
        else:
            raise Exception('Wrong task string structure')


    '''
        's' for starting a task
        'f' for ending a task and load the next task
        'c' for cancelling the current task (after which 's' must be pressed again to start the task)
        'o' for overtime 
        '.' for crash 
    '''
    def on_key(self, key):
        ## Quit if esc is pressed
        if key == keyboard.Key.esc:
            return False
        elif key.char == 's':
            self._task_state = 'started'
            if not self._is_training:
                self.__write_to_res_file('start')
        elif key.char == 'f':
            ## write to record and display the next task target (but do not start it)
            self._task_state = 'finished'
            if not self._is_training:
                self.__write_to_res_file('finish')
            ## If OpenCV would not work we switch to PIL
            self._task_ind += 1
            if self._task_ind < len(self._tasks) and self._tid == '1':
                ## Only load new target for the first task
                self.__load_target_img(self._tasks[self._task_ind][1])
            else:
                ## Done with the study
                return False
        elif key.char == 'c':
            self._task_state = 'cancelled'
            if not self._is_training:
                self.__write_to_res_file('cancelled')
        elif key.char == '.':
            ## TODO: load a backup target after crash
            self._task_state = 'crash'
            if not self._is_training:
                self.__write_to_res_file('crash')
        elif key.char == 'o':
            self._task_state =  'overtime'
            if not self._is_training:
                self.__write_to_res_file('overtime')
        print('Task state %s' % self._task_state)
        print('Current task: Technique %s Size %s Rep %s' % (self.__parse_task_str(self._tasks[self._task_ind][0])))
        print('Commands: (s) start (f) finish with success (c) cancel (.) report crash (o) report overtime (esc) QUIT')

    '''
    close all open files 
    '''
    def end(self):
        self._vconn.disconnect()
        self._vicon_file.close()
        self._res_file.close()
        
arg_parser = argparse.ArgumentParser()
arg_parser.add_argument('--participant', help='Participant id (1-8)')
arg_parser.add_argument('--task', help='1 for single-target task, 2 for sequence task')        
arg_parser.add_argument('--training', action='store_true', help='1 for single-target task, 2 for sequence task')        
arg_parser.add_argument('--startfrom', type=int, help='start from the middle of a task sequence')        
args = arg_parser.parse_args()
if args.startfrom is None:
    study = StudyRunner(args.task_id, args.participant_id, args.training)
else:
    study = StudyRunner(args.task_id, args.participant_id, args.training, args.startfrom)

study.start()
