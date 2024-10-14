import math

class ServoAngleConversion():

    def __init__(self):
        
        self.RADIANS_PER_ENCODER_TICK = 240 / 360 * (math.pi * 2) / 1000 #pulse width -----> radian
        self.ENCODER_TICKS_PER_RADIAN = 1 / self.RADIANS_PER_ENCODER_TICK #radians ----> pulse
        # self.ENCODER_RESOLUTION = 1000
        # self.MAX_POSITION = self.ENCODER_RESOLUTION - 1
        # self.VELOCITY_PER_TICK = 10
        # self.MAX_VELOCITY = 100
        # self.MIN_VELOCITY = self.VELOCITY_PER_TICK

        self.min_angle_raw = 1000
        self.max_angle_raw = 0
        self.initial_position_raw = 500

        self.flipped = self.min_angle_raw > self.max_angle_raw
        if self.flipped:
            self.min_angle = (self.initial_position_raw - self.min_angle_raw) * self.RADIANS_PER_ENCODER_TICK
            self.max_angle = (self.initial_position_raw - self.max_angle_raw) * self.RADIANS_PER_ENCODER_TICK
        else:
            self.min_angle = (self.min_angle_raw - self.initial_position_raw) * self.RADIANS_PER_ENCODER_TICK
            self.max_angle = (self.max_angle_raw - self.initial_position_raw) * self.RADIANS_PER_ENCODER_TICK

        self.min_pulse = min(self.min_angle_raw, self.max_angle_raw)
        self.max_pulse = max(self.min_angle_raw, self.max_angle_raw)

    def rad_to_pulse(self, angle, initial_position_raw, flipped, encoder_ticks_per_radian):
        angle_raw = angle * encoder_ticks_per_radian
        return initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw

    def pulse_to_rad(self, raw, initial_position_raw, flipped, radians_per_encoder_tick):
        return (initial_position_raw - raw if flipped else raw - initial_position_raw) * radians_per_encoder_tick

    def pos_rad_to_pulse(self, pos_rad): #radians ---> pulse width
        if pos_rad < self.min_angle:
            pos_rad = self.min_angle
        elif pos_rad > self.max_angle:
            pos_rad = self.max_angle
        return self.rad_to_pulse(pos_rad, self.initial_position_raw, self.flipped, self.ENCODER_TICKS_PER_RADIAN)

    def pos_pulse_to_rad(self, pos_pulse): #Pulse width ----> radians
        if pos_pulse < self.min_angle:
            pos_pulse = self.min_pulse
        elif pos_pulse > self.max_pulse:
            pos_pulse = self.max_pulse
        return self.pulse_to_rad(pos_pulse, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)

    def rad_to_deg(self, radians):
        return radians * (180 / math.pi)

    def deg_to_rad(self, degree):
        return degree * (math.pi / 180)

    def pos_pulse_to_deg(self, pos_pulse):
        rad = self.pos_pulse_to_rad(pos_pulse)
        deg = self.rad_to_deg(rad)

        norm = deg % 360

        min_deg = self.rad_to_deg(self.min_angle) % 360
        max_deg = self.rad_to_deg(self.max_angle) % 360

        return max(min(norm, max_deg), min_deg)
        # return deg % 360


    

# if __name__ == '__main__':
#         servoAngle = ServoAngleConversion()
#         print(servoAngle.pos_pulse_to_rad(0))
#         # print(servoAngle.pos_rad_to_pulse(2.09))
#         print(servoAngle.pos_pulse_to_deg(240))



