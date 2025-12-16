import math
from enum import Enum

class DIR(Enum):
    FORWARD = 1
    STOP = 0
    REVERSE = -1


class MotorDriver:
    """Mock MotorDriver for testing without hardware"""
    
    # Beide Motoren werden mit unterschiedlichen PWM angesteuert
    pin_FmotL = "P9_14" # GPIO 50 PWMA linker Motor
    pin_FmotR = "P9_21" # GPIO 51 PWMB rechter Motor

    pin_BmotL = "P9_16" # GPIO 50 PWMA linker Motor
    pin_BmotR = "P9_22" # GPIO 51 PWMB rechter Motor
    # Eingänge In2 und In2 invertiert zueinander (geben Drehrichtung vor)

    FREQ = 250000
    pwm_max = 90
    pwm_min = 30

    def __init__(self, v_max):
        """Initialize mock motor driver"""
        self.v_max = 0
        self.pwm_l = 0
        self.pwm_r = 0
        self._state = {
            self.pin_FmotL: 0,
            self.pin_FmotR: 0,
            self.pin_BmotL: 0,
            self.pin_BmotR: 0,
        }
        self.v_max = v_max
 




    def set_wheel_duty(self, ff_l, ff_r):
        """Set wheel duty cycle (mock version)"""
        self.pwm_l = math.floor(abs(ff_l)/100 * self.pwm_max)
        self.pwm_r = math.floor(abs(ff_r)/100 * self.pwm_max)

        # Reset all pins
        for pin in self._state:
            self._state[pin] = 0

        # left wheel
        if ff_l > 0:
            self._state[self.pin_FmotL] = self.pwm_l
            print(f"[MOCK] Left wheel forward: {self.pwm_l}%")
        elif ff_l < 0:
            self._state[self.pin_BmotL] = self.pwm_l
            print(f"[MOCK] Left wheel reverse: {self.pwm_l}%")
        else:
            print("[MOCK] Left wheel stop")

        # right wheel
        if ff_r > 0:
            self._state[self.pin_FmotR] = self.pwm_r
            print(f"[MOCK] Right wheel forward: {self.pwm_r}%")
        elif ff_r < 0:
            self._state[self.pin_BmotR] = self.pwm_r
            print(f"[MOCK] Right wheel reverse: {self.pwm_r}%")
        else:
            print("[MOCK] Right wheel stop")

    def get_state(self):
        """Return current mock motor state for testing"""
        return self._state.copy()

