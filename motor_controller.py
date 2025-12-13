
import time
import math
from motor_driver import MotorDriver
import kinematic as rk


class Controller:

    unit = 200#mm
    
    def __init__(self):
        self.v_max = rk.speeds[4]
        self.md = MotorDriver(self.v_max)
        self.linear_speed = rk.speeds[2]


    def clamp_speed(self, speed):
        clamped_speed = 0
        if(speed<=self.v_max and speed>=-self.v_max):
            clamped_speed = speed
        else:
            if(speed<-self.v_max):
                clamped_speed = -self.v_max
            else:
                clamped_speed = self.v_max

        return clamped_speed


    def move(self):
        vl = self.linear_speed-self.omega*rk.LwheelBase
        
    