import math

class ServoAngleConversion():

    def __init__(self):
        
        self.RADIANS_PER_ENCODER_TICK = 240 / 360 * (math.pi * 2) / 1000 #pulse width -----> radian
        self.ENCODER_TICKS_PER_RADIAN = 1 / self.RADIANS_PER_ENCODER_TICK #radians ----> pulse
        self.ENCODER_RESOLUTION = 1000
        self.MAX_POSITION = self.ENCODER_RESOLUTION - 1
        self.VELOCITY_PER_TICK = 10
        self.MAX_VELOCITY = 100
        self.MIN_VELOCITY = self.VELOCITY_PER_TICK

        self.min_angle_raw = 1000
        self.max_angle_raw = 0
        self.initial_position_raw = 500

        self.flipped = self.min_angle_raw > self.max_angle_raw
        if self.flipped:
            self.min_angle = ()



