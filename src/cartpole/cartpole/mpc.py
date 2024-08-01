from typing import Callable
import numpy as np
from abc import ABC, abstractmethod
import casadi as ca

from scipy.signal import StateSpace


class Cartpole(ABC):
    def __init__(self) -> None:
        self.m = 0.25
        self.M = 0.1
        self.b = 0.1
        self.l = 0.3
        self.I = 0.00801
        self.g = 9.81

    @abstractmethod
    def transition_fn(self, x, u):
        pass


class LinearCartpole(Cartpole):
    def __init__(self, dt) -> None:
        super().__init__()
        z = self.I * (self.M + self.m) + self.M * self.m * self.l**2

        A = np.array(
            [
                [0, 1, 0, 0],
                [
                    0,
                    -(self.I + self.m * self.l**2) * self.b / z,
                    -(self.m**2) * self.l**2 * self.g / z,
                    0,
                ],
                [0, 0, 0, 1],
                [
                    0,
                    self.m * self.l * self.b / z,
                    self.m * self.l * self.g * (self.m + self.m) / z,
                    0,
                ],
            ]
        )

        B = np.array(
            [
                [0],
                [(self.I + self.m * self.l**2) / z],
                [0],
                [-self.m * self.l / z],
            ]
        )

        C = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

        D = np.zeros((4, 1))

        self.sys = StateSpace(A, B, C, D).to_discrete(dt)

    def transition_fn(self, x: ca.DM | ca.SX, u: ca.DM | ca.SX) -> ca.DM | ca.SX:
        return ca.DM(self.sys.A) @ x + ca.DM(self.sys.B) * u


class NonlinearCartpole(Cartpole):
    def __init__(self, *args, **kwargs):
        super().__init__()

    def transition_fn(self, x: ca.DM | ca.SX, u: ca.DM | ca.SX) -> ca.DM | ca.SX:
        dx = x[1]
        theta = x[2]
        dtheta = x[3]

        beta = (
            self.I * (self.M + self.m)
            + self.M * self.m * self.l**2
            + ca.sin(theta) ** 2 * self.m**2 * self.l**2
        )

        return ca.vertcat(
            dx,
            (
                (self.I + self.m * self.l**2)
                * (self.m * self.l * dtheta**2 * ca.sin(theta) - self.b * dx)
                - self.m**2 * self.l**2 * self.g * ca.cos(theta) * ca.sin(theta)
                + (self.I + self.m * self.l**2) * u
            )
            / beta,
            dtheta,
            (
                self.m
                * self.l
                * ca.cos(theta)
                * (self.b * dx - self.m * self.l * dtheta**2 * ca.sin(theta))
                + (self.M + self.m) * self.m * self.l * self.g * ca.sin(theta)
                - self.m * self.l * ca.cos(dtheta) * u
            )
            / beta,
        )


class CartpoleMPC:
    def __init__(
        self,
        f: Callable[[ca.DM | ca.SX, ca.DM | ca.SX], ca.DM | ca.SX],
        nonlinear: bool = False,
        dt: float | None = None,
    ) -> None:
        N = 25
        self.num_states = 4
        self.num_inputs = 1

        opti = ca.Opti()
        self.x = opti.variable(self.num_states, N + 1)
        self.u = opti.variable(self.num_inputs, N)
        self.x0 = opti.parameter(self.num_states)
        self.r = opti.parameter(self.num_states)

        self.set_point = 0.0

        J = 0

        Q = np.diag([0.0, 0.0, 1.0, 0.0])  # state weighing matrix
        R = np.diag([0.1])  # controls weighing matrix

        for k in range(N):
            J += (self.x[:, k] - self.r).T @ Q @ (self.x[:, k] - self.r) + self.u[
                :, k
            ].T @ R @ self.u[:, k]
            x_next = (
                f(self.x[:, k], self.u[:, k])
                if not nonlinear
                else self.x[:, k] + f(self.x[:, k], self.u[:, k]) * dt
            )
            opti.subject_to(self.x[:, k + 1] == x_next)

        opti.minimize(J)
        opti.subject_to(self.x[:, 0] == self.x0)
        opti.subject_to(self.u[0, :] >= -5)
        opti.subject_to(self.u[0, :] <= 5)

        opti.set_value(self.r, ca.vertcat(0, 0, self.set_point, 0))

        p_opts = {
            "expand": True,
        }
        s_opts = {
            "max_iter": 1000,
            "print_level": 0,
            "acceptable_tol": 1e-8,
            "acceptable_obj_change_tol": 1e-6,
        }

        opti.solver("ipopt", p_opts, s_opts)
        self.opti = opti

    def compute_control(self, x: ca.DM) -> None:
        try:
            self.opti.set_value(self.x0, x)
            sol = self.opti.solve()
            return sol.value(self.u)[0]
        except RuntimeError:
            return [0.0]
