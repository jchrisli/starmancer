'''
Implementation of the dynamic low-pass filter in 'Distant freehand pointing and clicking on very large, high resolution displays'
UIST 05
'''
import numpy as np

'''
    A little hack for maintaining reference to immutable types in Python
'''
class Freq(object):
    def __init__(self):
        self._val = 1

    def get(self):
        return self._val

    def set(self, val):
        self._val = val

class Filter(object):
    def __init__(self, update_freq):
        self._update_freq = update_freq
        self._first_time = True
        self._prev_val = None

    def clear(self):
        self._first_time = True

    def set_update_frequency(self, update_freq):
        self._update_freq = update_freq


class LowPassFilter(Filter):
    def __init__(self, update_freq):
        super(LowPassFilter, self).__init__(update_freq)
        self._cut_off_freq = 1

    def apply(self, new_val):
        if self._first_time:
            self._prev_val = new_val
            self._first_time = False

        update_freq = self._update_freq.get()
        te = 1.0 / update_freq
        tau = 1.0 / (2.0 * 3.14159265 * self._cut_off_freq)

        filtered_val = new_val + (tau / te) * self._prev_val * (1.0 / (1.0 + tau / te))
        self._prev_val = filtered_val
        return filtered_val

    def set_cutoff_freq(self, f):
        self._cut_off_freq = f

    def get_cutoff_freq(self):
        return self._cut_off_freq
        
class LowPassDynamicFilter(LowPassFilter):
    def __init__(self, update_freq):
        super(LowPassDynamicFilter, self).__init__(update_freq)
        self._cut_off_freq_high = 2
        self._velocity_low = None
        self._velocity_high = None
        self._velocity_filter = LowPassFilter(update_freq)
        self._last_position_for_velocity = None

    def set_cutoff_freq_low(self, freq):
        self._cut_off_freq = freq
        self.set_cutoff_freq_velocity()

    def set_cutoff_freq_high(self, freq):
        self._cut_off_freq_high = freq
        self.set_cutoff_freq_velocity()

    def set_velocity_low(self, vel):
        self._velocity_low = vel

    def set_velocity_high(self, vel):
        self._velocity_high = vel

    def set_cutoff_freq_velocity(self):
        self._velocity_filter.set_cutoff_freq(self._cut_off_freq + 0.75 * (self._cut_off_freq_high - self._cut_off_freq))

    def apply(self, point_data):
        """Apply the filter to an incoming data point
        
        Arguments:
            point_data {[1d 3-element array]} -- [income 3-element data]
        """
        point_data = np.array(point_data)
        if self._first_time:
            self._prev_val = point_data
            self._last_position_for_velocity = point_data
            self._first_time = False

        update_freq = self._update_freq.get()

	    ## first get an estimate of velocity (with filter)
        position_for_velocity = self._velocity_filter.apply(point_data)
        vel = update_freq * (position_for_velocity - self._last_position_for_velocity)
        self._last_position_for_velocity = position_for_velocity
        vel = np.abs(vel)

        t = (vel - np.array([self._velocity_low for i in range(0, 3)])) / (self._velocity_high - self._velocity_low)
        t = np.clip(t, 0.0, 1.0)

        cutoff = (self._cut_off_freq_high * t) + (self._cut_off_freq * (np.array([1, 1, 1]) - t))
        
        ## the sampling period (in seconds)
        te = np.array([1.0 / update_freq, 1.0 / update_freq, 1.0 / update_freq]) 
        ## a time constant calculated from the cut-off frequency
        tau = np.divide(np.full(3, 1.0 / 2 / 3.14159265), cutoff)

        #filtered_val = point_data + np.multiply(np.divide(tau, te), self._prev_val)
        #filtered_val = np.multiply(filtered_val 
        filtered_val_x = (point_data[0] + tau[0] / te[0] * self._prev_val[0]) * (1.0 / (1.0 + tau[0] / te[0]))
        filtered_val_y = (point_data[1] + tau[1] / te[1] * self._prev_val[1]) * (1.0 / (1.0 + tau[1] / te[0]))
        filtered_val_z = (point_data[2] + tau[2] / te[2] * self._prev_val[2]) * (1.0 / (1.0 + tau[2] / te[0]))
        filtered_val = [filtered_val_x, filtered_val_y, filtered_val_z]

        self._prev_val = np.array(filtered_val)

        return filtered_val
        