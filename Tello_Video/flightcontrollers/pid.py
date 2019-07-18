import math

class PidController:
    def __inti__(self, angle, Kp, Ki, Kd):
        self.angle_ = angle # True if we're controlling an angle [-pi, pi]
        self.target_ = 0
        self.prev_error_ = 0
        self.integral_ = 0
        self.Kp_ = Kp
        self.Ki_ = Ki
        self.Kd_ = Kd

    # Intuitive constructor
    #Controller(bool angle, double damping_ratio, double natural_frequency)
    #{
    #  angle_ = angle;
    #  Kp_ = natural_frequency * natural_frequency * (1 + 2 * damping_ratio);
    #  Ki_ = natural_frequency * natural_frequency * natural_frequency;
    #  Kd_ = natural_frequency * (1 + 2 * damping_ratio);
    #}

    # Set target
    def set_target(self, target):
        self.target_ = target
        self.prev_error_ = 0
        self.integral_ = 0

    def set_coefficients(self, Kp, Ki, Kd):
        self.Kp_ = Kp
        self.Ki_ = Ki
        self.Kd_ = Kd

    def target(self):
        return self.target_

    # Run one calculation
    def calc(self, state, dt, bias):
        error = self.target_ - state

        if self.angle_:
            # Deal with discontinuity
            while (error < -math.pi):
                error = error + 2 * math.pi
            
            while (error > math.pi):
                error = error - 2 * math.pi

        self.integral_ = self.integral_ + (error * dt)
        derivative = (error - self.prev_error_) / dt
        self.prev_error_ = error

        return self.Kp_ * error + self.Ki_ * self.integral_ + self.Kd_ * derivative + bias

    @staticmethod
    def clamp(variableToClamp):
        OutputMin = -1.0
        OutputMax = 1.0
        if variableToClamp <= OutputMin: 
            return OutputMin
        if variableToClamp >= OutputMax:
            return OutputMax
        return variableToClamp


class PidController2:

    def __init__(self, angle, Kp, Kd):
        self.angle_ = angle
        self.Kp_ = Kp
        self.Kd_ = Kd

    def set_coefficients(self, Kp, Kd):
        self.Kp_ = Kp
        self.Kd_ = Kd

        # Run one calculation
    def calc(self, y_target, y_actual, y_dot_target, y_dot_actual):
        y_error = y_target - y_actual
        y_dot_error = y_dot_target - y_dot_actual

        if self.angle_:
            # Deal with discontinuity
            while y_error < -math.pi: 
                y_error = y_error + 2 * math.pi
           
            while y_error > math.pi: 
                y_error = y_error - 2 * math.pi

        return self.Kp_ * y_error + self.Kd_ * y_dot_error