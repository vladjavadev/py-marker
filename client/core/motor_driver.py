import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import math
from enum import Enum

class DIR(Enum):
    FORWARD = 1
    STOP = 0
    REVERSE = -1


class MotorDriver:
    """Real MotorDriver for BeagleBone Black hardware"""
    
    # Beide Motoren werden mit unterschiedlichen PWM angesteuert
    pin_FmotL = "P9_14"  # GPIO 50 PWMA linker Motor (Forward)
    pin_FmotR = "P9_21"  # GPIO 51 PWMB rechter Motor (Forward)

    pin_BmotL = "P9_16"  # GPIO 50 PWMA linker Motor (Backward)
    pin_BmotR = "P9_22"  # GPIO 51 PWMB rechter Motor (Backward)
    # Eingänge In2 und In2 invertiert zueinander (geben Drehrichtung vor)

    FREQ = 250000
    pwm_max = 90
    pwm_min = 30

    def __init__(self, v_max):
        """Initialize motor driver with hardware PWM
        
        Args:
            v_max: Maximum velocity in mm/s
        """
        # Start all PWM channels at 0% duty cycle
        PWM.start(self.pin_FmotL, 0, self.FREQ, 0)
        PWM.start(self.pin_FmotR, 0, self.FREQ, 0)
        PWM.start(self.pin_BmotL, 0, self.FREQ, 0)
        PWM.start(self.pin_BmotR, 0, self.FREQ, 0)
        
        self.v_max = v_max
        self.pwm_l = 0
        self.pwm_r = 0

    def set_wheel_duty(self, duty_l, duty_r):
        """Set wheel duty cycle from PID controller output
        
        Args:
            duty_l: Left wheel duty cycle (-100 to 100)
            duty_r: Right wheel duty cycle (-100 to 100)
        """
        # Clamp input values to valid range
        duty_l = max(-100, min(100, duty_l))
        duty_r = max(-100, min(100, duty_r))
        
        # Calculate PWM values (map from 0-100% to pwm_min-pwm_max range)
        if abs(duty_l) > 0:
            self.pwm_l = int(self.pwm_min + (abs(duty_l) / 100.0) * (self.pwm_max - self.pwm_min))
        else:
            self.pwm_l = 0
            
        if abs(duty_r) > 0:
            self.pwm_r = int(self.pwm_min + (abs(duty_r) / 100.0) * (self.pwm_max - self.pwm_min))
        else:
            self.pwm_r = 0

        # Left motor control
        if duty_l > 0:  # Forward
            PWM.set_duty_cycle(self.pin_BmotL, 0)
            PWM.set_duty_cycle(self.pin_FmotL, self.pwm_l)
        elif duty_l < 0:  # Reverse
            PWM.set_duty_cycle(self.pin_FmotL, 0)
            PWM.set_duty_cycle(self.pin_BmotL, self.pwm_l)
        else:  # Stop
            PWM.set_duty_cycle(self.pin_FmotL, 0)
            PWM.set_duty_cycle(self.pin_BmotL, 0)

        # Right motor control
        if duty_r > 0:  # Forward
            PWM.set_duty_cycle(self.pin_BmotR, 0)
            PWM.set_duty_cycle(self.pin_FmotR, self.pwm_r)
        elif duty_r < 0:  # Reverse
            PWM.set_duty_cycle(self.pin_FmotR, 0)
            PWM.set_duty_cycle(self.pin_BmotR, self.pwm_r)
        else:  # Stop
            PWM.set_duty_cycle(self.pin_FmotR, 0)
            PWM.set_duty_cycle(self.pin_BmotR, 0)

    def set_motor_speed(self, vl, vr):
        """Set motor speed in mm/s (legacy method for compatibility)
        
        Args:
            vl: Left wheel velocity in mm/s
            vr: Right wheel velocity in mm/s
        """
        # Convert velocity to duty cycle percentage
        if self.v_max == 0:
            duty_l = 0
            duty_r = 0
        else:
            duty_l = (vl / self.v_max) * 100.0
            duty_r = (vr / self.v_max) * 100.0
        
        # Use the main control method
        self.set_wheel_duty(duty_l, duty_r)

    def stop(self):
        """Emergency stop - set all motors to 0"""
        PWM.set_duty_cycle(self.pin_FmotL, 0)
        PWM.set_duty_cycle(self.pin_FmotR, 0)
        PWM.set_duty_cycle(self.pin_BmotL, 0)
        PWM.set_duty_cycle(self.pin_BmotR, 0)
        self.pwm_l = 0
        self.pwm_r = 0

    def cleanup(self):
        """Clean up PWM resources"""
        self.stop()
        PWM.stop(self.pin_FmotL)
        PWM.stop(self.pin_FmotR)
        PWM.stop(self.pin_BmotL)
        PWM.stop(self.pin_BmotR)
        PWM.cleanup()


# Test code for hardware verification
if __name__ == "__main__":
    import time
    
    print("=== Testing MotorDriver on BeagleBone ===")
    print("WARNING: Motors will run! Make sure robot is on blocks or safely secured.\n")
    
    try:
        driver = MotorDriver(v_max=200)
        
        # Test 1: Forward motion at 50%
        print("Test 1: Forward motion (50% duty) for 2 seconds")
        driver.set_wheel_duty(50, 50)
        time.sleep(2)
        
        # Test 2: Stop
        print("Test 2: Stop for 1 second")
        driver.stop()
        time.sleep(1)
        
        # Test 3: Turn left
        print("Test 3: Turn left (30% left, 70% right) for 2 seconds")
        driver.set_wheel_duty(30, 70)
        time.sleep(2)
        
        # Test 4: Stop
        print("Test 4: Stop for 1 second")
        driver.stop()
        time.sleep(1)
        
        # Test 5: Reverse
        print("Test 5: Reverse (-50% duty) for 2 seconds")
        driver.set_wheel_duty(-50, -50)
        time.sleep(2)
        
        # Test 6: Stop
        print("Test 6: Stop for 1 second")
        driver.stop()
        time.sleep(1)
        
        # Test 7: Rotate in place
        print("Test 7: Rotate in place for 2 seconds")
        driver.set_wheel_duty(-50, 50)
        time.sleep(2)
        
        # Final stop
        print("Test complete - stopping motors")
        driver.stop()
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        driver.stop()
    except Exception as e:
        print(f"\n\nError during test: {e}")
        driver.stop()
    finally:
        print("Cleaning up PWM resources")
        driver.cleanup()