
class PID:
    def __init__(self, kp: float, ki: float = 0.0, kd: float = 0.0, out_min: float = -100.0, out_max: float = 100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self._integral = 0.0
        self._last_err = None


    def reset(self):
        self._integral = 0.0
        self._last_err = None

    def update(self, err: float, dt: float) -> float:
        if dt <= 0:
            return 0.0

        self._integral += err * dt
        deriv = 0.0
        if self._last_err is not None:
            deriv = (err - self._last_err) / dt
        self._last_err = err

        out = self.kp * err + self.ki * self._integral + self.kd * deriv
        if out > self.out_max:
            out = self.out_max
        if out < self.out_min:
            out = self.out_min
        return out

