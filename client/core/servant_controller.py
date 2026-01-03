import time
from typing import Optional

import client.core.kinematic as rk
from client.core.mock_motor_driver import MotorDriver
from client.core.pid_controller import PID


class SlaveController:
    """Controls a differential-drive slave robot wheels using PID on wheel speed.

    - Accepts target linear velocity (mm/s) and angular velocity (rad/s).
    - Converts to left/right wheel targets using the kinematic model.
    - Produces signed duty cycles in range -100..100 and sends to motor driver.
    - If measured wheel speeds are provided to `update()` it runs PID correction.
    """

    def __init__(self, kp=1.1, ki=0.1, kd=0.6, vMode=3):
        self.v_max = rk.speeds[3]
        self.max_duty = rk.duty_list[3] 
        self.rd = MotorDriver(self.max_duty)
        # per-wheel PIDs
        self.motor_pid = PID(kp, ki, kd, out_min=-50, out_max=50.0)
        self.base_duty = 30 
        self.delta_error = 0.0
        self._last_time = None

    def set_target_duty(self, duty_l: float, duty_r: float ):
        self.rd.set_wheel_duty(duty_l, duty_r)
        


    def set_delta_error(self, delta_error: float):
        self.delta_error = delta_error

    def update(self):
        """Compute and apply wheel duties.

        If measured wheel speeds (mm/s) are provided, PID will be applied
        to correct the feedforward duty. Otherwise a pure feedforward duty
        mapping is used.
        """
        now = time.time()
        if self._last_time is None:
            dt = 0.02
        else:
            dt = now - self._last_time
        self._last_time = now

        correction = self.motor_pid.update(self.delta_error, dt)


        # combine feedforward percent and PID correction (both are in percent units)
        out_l = self.base_duty + correction
        out_r = self.base_duty - correction

        # clamp
        if out_l > self.max_duty:
            out_l = self.max_duty
        if out_l < -self.max_duty:
            out_l = 0
        if out_r > self.max_duty:
            out_r = self.max_duty
        if out_r < -self.max_duty:
            out_r = 0

        self.duty_l = out_l
        self.duty_r = out_r

        self.rd.set_wheel_duty(out_l, out_r)
        return out_l, out_r


if __name__ == "__main__":
    # simple demonstration: drive forward for 2 seconds using feedforward only
    sc = SlaveController(3)
    # set 1/2 of max forward speed
    linear = sc.v_max * 0.5
    omega = 0.0
    sc.set_target_velocity(linear, omega)
    sc.update()  # feedforward
    time.sleep(2.0)
    sc.set_target_velocity(0.0, 0.0)
    sc.update()