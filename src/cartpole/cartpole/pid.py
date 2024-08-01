import numpy as np


class CartpolePID:
    def __init__(
        self,
        dt: float,
        kp: float,
        ki: float,
        kd: float,
        set_point: float,
        sum_constraint: tuple[int, int] = (-1.0, 1.0),
    ) -> None:
        self.dt = dt
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.sum_constraint = sum_constraint
        self.set_point = set_point

        self.error_sum = 0
        self.error_prev = 0

    def compute_control(self, error: float) -> float:
        self.error_sum += error
        self.error_sum = np.clip(
            self.error_sum, self.sum_constraint[0], self.sum_constraint[1]
        )

        u = (
            self.kp * error
            + self.ki * self.error_sum * self.dt
            + self.kd * (error - self.error_prev) / self.dt
        )

        self.error_prev = error

        return u
