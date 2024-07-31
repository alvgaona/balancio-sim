import rclpy
import traceback
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState

from abc import ABC, abstractmethod
import numpy as np
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

    def transition_fn(self, x, u):
        return ca.DM(self.sys.A) @ x + ca.DM(self.sys.B) * u


class NonlinearCartpole(Cartpole):
    def __init__(self, *args, **kwargs):
        super().__init__()

    def transition_fn(self, x, u):
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
    def __init__(self, f, nonlinear: bool = False, dt: float | None = None) -> None:
        N = 25
        self.num_states = 4
        self.num_inputs = 1

        opti = ca.Opti()
        self.x = opti.variable(self.num_states, N + 1)
        self.u = opti.variable(self.num_inputs, N)
        self.x0 = opti.parameter(self.num_states)
        self.r = opti.parameter(self.num_states)

        J = 0

        Q = np.diag([0.0, 0.0, 50000.0, 0.0])  # state weighing matrix
        R = np.diag([0.01])  # controls weighing matrix

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
        opti.subject_to(self.u[0, :] >= -20)
        opti.subject_to(self.u[0, :] <= 20)

        opti.set_value(self.r, ca.vertcat(0, 0, 0, 0))

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
        self.k = 0

    def compute_control(self, x) -> None:
        try:
            self.opti.set_value(self.x0, ca.vertcat(0, 0, x, 0))
            sol = self.opti.solve()
            return sol.value(self.u)
        except RuntimeError:
            print(traceback.format_exc())
            return [0.0]


class CartpolePID:
    def __init__(
        self,
        dt: float,
        kp: float,
        ki: float,
        kd: float,
        sum_constraint: tuple[int, int] = (-1.0, 1.0),
    ) -> None:
        self.dt = dt
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.sum_constraint = sum_constraint
        self.error_sum = 0
        self.error_prev = 0

    def compute_control(self, theta: float, set_point: float) -> float:
        error = theta - set_point

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


class CartpoleNode(Node):
    def __init__(self):
        super().__init__("cartpole_node")
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.dt = 0.01

        # self.model = NonlinearCartpole()
        # self.controller = CartpolePID(
        #     f=self.model.transition_fn, nonlinear=True, dt=self.dt
        # )

        self.controller = CartpolePID(dt=self.dt, kp=50.0, ki=1.0, kd=5.0)

        self.set_point = 0.0

        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.joint_states_cb, 1
        )
        self.publisher = self.create_publisher(JointState, "/joint_command", 10)
        self.timer = self.create_timer(self.dt, self.pub_cart_force)

        self.pole_angle = np.pi

    def joint_states_cb(self, msg: JointState) -> None:
        self.pole_angle = np.pi - np.mod(msg.position[1], 2 * np.pi)

        self.get_logger().debug(f" Pole angle [rad]: {self.pole_angle}")

        error = self.pole_angle - self.set_point

        self.get_logger().debug(f" Error: {error}")

    def pub_cart_force(self):
        u = self.controller.compute_control(self.pole_angle, self.set_point)

        joint_state_msg = JointState()

        # TODO: fill the rest of the fields
        joint_state_msg.name.extend(["cartJoint"])
        joint_state_msg.effort.append(u)

        self.publisher.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    cartpole_node = CartpoleNode()
    rclpy.spin(cartpole_node)
    cartpole_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
