import math
import time


class PIDController:
    __name__ = "PIDController"
    min_out_limit, max_out_limit = -1.0, 1.0
    kp, ki, kd = 0.025, 0.001, 0.1
    proportional_term, integral_term, derivative_term = 0.0, 0.0, 0.0
    last_value, last_time = 0.0, 0.0
    time_sample = 0.025

    def __init__(self):
        super(PIDController, self).__init__()

    def limit_value(self, value):
        if value > self.max_out_limit:
            return self.max_out_limit
        else:
            return math.max(value, self.min_out_limit)

    def calc_pid(self, current_value, limit_value):
        now = float(time.time())
        change_in_time = float(now - self.last_time)

        if change_in_time >= self.time_sample:
            error = limit_value - current_value
            change_in_values = current_value - self.last_value

            self.proportional_term = self.kp * error
            self.integral_term = self.limit_value(
                self.integral_term + self.ki * error
            )
            self.derivative_term = self.kd * -change_in_values

            self.last_value = current_value
            self.last_time = now

        return limit_value(
            self.proportional_term + self.ki * self.integral_term + self.derivative_term
        )

    def limit_output(self, min, max):
        """Restringe o valor de saída do cálculo PID"""
        if min > max:
            return
        self.min_out_limit = min
        self.max_out_limit = max
        self.integral_term = self.limit_value(self.integral_term)

    def adjust_pid(self, kp=None, ki=None, kd=None):
        """Ajuste dos parâmetros do cálculo PID
        Recebe valores novos se válidos, e retorna os valores"""
        if kp is None:
            return self.kp
        if ki is None:
            return self.ki
        if kd is None:
            return self.kd
        self.kp = kp if kp > 0 else __class__.kp
        self.ki = ki if ki > 0 else __class__.ki
        self.kd = kd if kd > 0 else __class__.kd
