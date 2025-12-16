class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        # Интеграл ошибки
        self.integral += error * dt
        # Производная ошибки
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        # PID формула
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        # Запоминаем ошибку
        self.prev_error = error
        return output