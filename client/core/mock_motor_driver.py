import math
from enum import Enum

class DIR(Enum):
    FORWARD = 1
    STOP = 0
    REVERSE = -1


class MotorDriver:
    """Mock MotorDriver for testing without hardware"""
    
    # Beide Motoren werden mit unterschiedlichen PWM angesteuert
    pin_FmotL = "P9_14"  # GPIO 50 PWMA linker Motor
    pin_FmotR = "P9_21"  # GPIO 51 PWMB rechter Motor

    pin_BmotL = "P9_16"  # GPIO 50 PWMA linker Motor
    pin_BmotR = "P9_22"  # GPIO 51 PWMB rechter Motor
    # Eingänge In2 und In2 invertiert zueinander (geben Drehrichtung vor)

    FREQ = 250000
    pwm_max = 90
    pwm_min = 30

    def __init__(self, v_max):
        """Initialize mock motor driver"""
        self.v_max = v_max  # Store v_max properly
        self.pwm_l = 0
        self.pwm_r = 0
        self._state = {
            self.pin_FmotL: 0,
            self.pin_FmotR: 0,
            self.pin_BmotL: 0,
            self.pin_BmotR: 0,
        }

    def set_wheel_duty(self, ff_l, ff_r):
        """Set wheel duty cycle (mock version)
        
        Args:
            ff_l: Left wheel duty cycle (-100 to 100)
            ff_r: Right wheel duty cycle (-100 to 100)
        """
        # Clamp input values to valid range
        ff_l = max(-100, min(100, ff_l))
        ff_r = max(-100, min(100, ff_r))
        
        # Calculate PWM values (map from 0-100% to pwm_min-pwm_max range)
        if abs(ff_l) > 0:
            self.pwm_l = int(self.pwm_min + (abs(ff_l) / 100.0) * (self.pwm_max - self.pwm_min))
        else:
            self.pwm_l = 0
            
        if abs(ff_r) > 0:
            self.pwm_r = int(self.pwm_min + (abs(ff_r) / 100.0) * (self.pwm_max - self.pwm_min))
        else:
            self.pwm_r = 0

        # Reset all pins
        for pin in self._state:
            self._state[pin] = 0

        # Left wheel
        if ff_l > 0:
            self._state[self.pin_FmotL] = self.pwm_l
            print("[MOCK] Left wheel forward: duty={:.1f}%, PWM={}".format(ff_l, self.pwm_l))
        elif ff_l < 0:
            self._state[self.pin_BmotL] = self.pwm_l
            print("[MOCK] Left wheel reverse: duty={:.1f}%, PWM={}".format(abs(ff_l), self.pwm_l))
        else:
            print("[MOCK] Left wheel stop")

        # Right wheel
        if ff_r > 0:
            self._state[self.pin_FmotR] = self.pwm_r
            print("[MOCK] Right wheel forward: duty={:.1f}%, PWM={}".format(ff_r, self.pwm_r))
        elif ff_r < 0:
            self._state[self.pin_BmotR] = self.pwm_r
            print("[MOCK] Right wheel reverse: duty={:.1f}%, PWM={}".format(abs(ff_r), self.pwm_r))
        else:
            print("[MOCK] Right wheel stop")

    def get_state(self):
        """Return current mock motor state for testing"""
        return self._state.copy()


# Test code
if __name__ == "__main__":
    print("=== Testing MotorDriver ===\n")
    
    driver = MotorDriver(v_max=200)
    
    # Test 1: Forward motion
    print("Test 1: Forward motion (50% duty)")
    driver.set_wheel_duty(50, 50)
    print("State: {}\n".format(driver.get_state()))
    
    # Test 2: Turn left (right wheel faster)
    print("Test 2: Turn left")
    driver.set_wheel_duty(30, 70)
    print("State: {}\n".format(driver.get_state()))
    
    # Test 3: Reverse
    print("Test 3: Reverse")
    driver.set_wheel_duty(-50, -50)
    print("State: {}\n".format(driver.get_state()))
    
    # Test 4: Rotate in place
    print("Test 4: Rotate in place (one forward, one reverse)")
    driver.set_wheel_duty(-50, 50)
    print("State: {}\n".format(driver.get_state()))
    
    # Test 5: Stop
    print("Test 5: Stop")
    driver.set_wheel_duty(0, 0)
    print("State: {}\n".format(driver.get_state()))
    
    # Test 6: Maximum duty
    print("Test 6: Maximum duty (100%)")
    driver.set_wheel_duty(100, 100)
    print("State: {}\n".format(driver.get_state()))
    
    # Test 7: Low duty
    print("Test 7: Low duty (10%)")
    driver.set_wheel_duty(10, 10)
    print("State: {}\n".format(driver.get_state()))