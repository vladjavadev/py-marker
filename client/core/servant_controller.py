import time
from typing import Optional

import core.kinematic as rk
from core.motor_driver import MotorDriver
from core.pid_controller import PID


class SlaveController:
    """Controls a differential-drive slave robot wheels using PID on wheel speed.

    - Accepts target linear velocity (mm/s) and angular velocity (rad/s).
    - Converts to left/right wheel targets using the kinematic model.
    - Produces signed duty cycles in range -100..100 and sends to motor driver.
    - If measured wheel speeds are provided to `update()` it runs PID correction.
    """

    def __init__(self, kp=0.6, ki=0.1, kd=0.01, v_max: Optional[float] = None):
        self.rd = MotorDriver(v_max)
        self.v_max = v_max if v_max is not None else rk.speeds[-1]
        # per-wheel PIDs
        self.pid_l = PID(kp, ki, kd, out_min=-100.0, out_max=100.0)
        self.pid_r = PID(kp, ki, kd, out_min=-100.0, out_max=100.0)
        self.target_vl = 0.0
        self.target_vr = 0.0
        self._last_time = None

    def set_target_velocity(self, linear_v: float, angular_omega: float):
        """Set desired chassis linear velocity (mm/s) and angular velocity (rad/s)."""
        L = rk.LwheelBase
        # differential drive inverse kinematics
        self.target_vl = linear_v - angular_omega * (L / 2.0)
        self.target_vr = linear_v + angular_omega * (L / 2.0)

    def _vel_to_duty(self, v: float) -> float:
        # feedforward mapping: scale by v_max to get percentage
        if self.v_max == 0:
            return 0.0
        pct = (abs(v) / self.v_max) * 100.0
        if pct > 100.0:
            pct = 100.0
        return pct if v >= 0 else -pct

    def update(self, measured_vl: Optional[float] = None, measured_vr: Optional[float] = None):
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

        # feedforward duties
        ff_l = self._vel_to_duty(self.target_vl)
        ff_r = self._vel_to_duty(self.target_vr)

        if measured_vl is None or measured_vr is None:
            # no feedback available, use feedforward mapping
            self.rd.set_wheel_duty(ff_l, ff_r)
            return ff_l, ff_r

        # compute PID corrections (target - measured)
        err_l = self.target_vl - measured_vl
        err_r = self.target_vr - measured_vr

        corr_l = self.pid_l.update(err_l, dt)
        corr_r = self.pid_r.update(err_r, dt)

        # combine feedforward percent and PID correction (both are in percent units)
        out_l = ff_l + corr_l
        out_r = ff_r + corr_r

        # clamp
        if out_l > 100:
            out_l = 100.0
        if out_l < -100:
            out_l = -100.0
        if out_r > 100:
            out_r = 100.0
        if out_r < -100:
            out_r = -100.0

        self.rd.set_wheel_duty(out_l, out_r)
        return out_l, out_r


if __name__ == "__main__":
    # simple demonstration: drive forward for 2 seconds using feedforward only
    sc = SlaveController(v_max=200)
    # set 1/2 of max forward speed
    linear = sc.v_max * 0.5
    omega = 0.0
    sc.set_target_velocity(linear, omega)
    sc.update()  # feedforward
    time.sleep(2.0)
    sc.set_target_velocity(0.0, 0.0)
    sc.update()