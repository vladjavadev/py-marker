
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import math
from enum import Enum

class DIR(Enum):
    FORWARD = 1
    STOP = 0
    REVERSE =-1



class MotorDriver:
    
    # Beide Motoren werden mit unterschiedlichen PWM angesteuert
    pin_FmotL = "P9_14" # GPIO 50 PWMA linker Motor
    pin_FmotR = "P9_21" # GPIO 51 PWMB rechter Motor

    pin_BmotL = "P9_16" # GPIO 50 PWMA linker Motor
    pin_BmotR = "P9_22" # GPIO 51 PWMB rechter Motor
    # Eingänge In2 und In2 invertiert zueinander (geben Drehrichtung vor)

    FREQ = 250000
    pwm_max = 90
    pwm_min = 30

    def init(self, v_max):
        PWM.start(self.pin_FmotL, 0, self.FREQ, 0)
        PWM.start(self.pin_FmotR, 0, self.FREQ, 0)
        PWM.start(self.pin_BmotL, 0, self.FREQ, 0)
        PWM.start(self.pin_BmotR, 0, self.FREQ, 0)
        self.v_max = v_max
        self.pwm_l = 0
        self.pwm_r = 0


    def set_motor_speed(self, vl, vr):
        self.pwm_l = math.floor((abs(vl)/self.v_max)*self.pwm_max)
        self.pwm_r = math.floor((abs(vr)/self.v_max)*self.pwm_max)

        # left motor
        if vl > 0:
            PWM.set_duty_cycle(self.pin_BmotL, 0)
            PWM.set_duty_cycle(self.pin_FmotL, self.pwm_l)
        elif vl < 0:
            PWM.set_duty_cycle(self.pin_FmotL, 0)
            PWM.set_duty_cycle(self.pin_BmotL, self.pwm_l)
        else:
            PWM.set_duty_cycle(self.pin_FmotL, 0)
            PWM.set_duty_cycle(self.pin_BmotL, 0)

        # right motor
        if vr > 0:
            PWM.set_duty_cycle(self.pin_BmotR, 0)
            PWM.set_duty_cycle(self.pin_FmotR, self.pwm_r)
        elif vr < 0:
            PWM.set_duty_cycle(self.pin_FmotR, 0)
            PWM.set_duty_cycle(self.pin_BmotR, self.pwm_r)
        else:
            PWM.set_duty_cycle(self.pin_FmotR, 0)
            PWM.set_duty_cycle(self.pin_BmotR, 0)

