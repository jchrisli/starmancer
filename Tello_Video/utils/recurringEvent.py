'''
From
https://stackoverflow.com/questions/12435211/python-threading-timer-repeat-function-every-n-seconds
'''

from threading import Thread

class RecurringEvent(Thread):
    def __init__(self, event, hFn, interval):
        Thread.__init__(self)
        self.stopped = event
        self._interval = interval
        self._hFn = hFn

    def run(self):
        while not self.stopped.wait(self._interval):
            ## call a function
            self._hFn()