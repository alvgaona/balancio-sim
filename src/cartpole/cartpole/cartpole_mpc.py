import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

from abc import ABC, abstractmethod
import numpy as np
import casadi as ca

from scipy.signal import StateSpace


class Cartpole(ABC):
    def __init__(self) -> None:
        self.m = 0.25
        self.M = 0.029 * 2
        self.b = 0.1
        self.l = 0.3
        self.I = 1 / 3 * self.m * self.l**2
        self.g = 9.8

    @abstractmethod
    def transition_fn(self, x, u):
        pass


class LinearCartpole(Cartpole):
    def __init__(self, dt) -> None:
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


class CartpoleMPC:
    def __init__(self, f) -> None:
        N = 20
        self.num_states = 4
        self.num_inputs = 1

        opti = ca.Opti()
        self.x = opti.variable(self.num_states, self.N + 1)
        self.u = opti.variable(self.num_inputs, self.N)
        self.x0 = opti.parameter(self.num_states)
        self.r = opti.parameter(self.num_states)

        J = 0

        Q = np.diag([0.0, 0.0, 2000.0, 0.0])  # state weighing matrix
        R = np.diag([0.01])  # controls weighing matrix

        for k in range(N):
            J += (self.x[:, k] - self.r).T @ Q @ (self.x[:, k] - self.r) + self.u[
                :, k
            ].T @ R @ self.u[:, k]
            x_next = f(self.x[:, k], self.u[:, k])
            opti.subject_to(self.x[:, k + 1] == x_next)

        opti.minimize(J)
        opti.subject_to(self.x[:, 0] == self.x0)
        opti.subject_to(self.u[0, :] >= -5)
        opti.subject_to(self.u[0, :] <= 5)

        # opti.set_value(self.x0, ca.vertcat(0.0, 0, np.pi, 0))
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
        self.opti.set_value(self.x0, ca.vertcat(0, 0, x, 0))
        sol = self.opti.solve()
        return sol.value(self.u)


class CartpoleNode(Node):
    def __init__(self):
        super().__init__("cartpole_node")
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.dt = 0.05

        self.model = LinearCartpole(self.dt)
        self.controller = CartpoleMPC(f=self.model.transition_fn)

        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.joint_states_cb, 1
        )
        self.publisher = self.create_publisher(JointState, "/joint_command", 10)
        # self.timer = self.create_timer(self.dt, self.pub_cart_force)

        self.pole_angle = 0

    def joint_states_cb(self, msg: JointState) -> None:
        self.pole_angle = np.mod(msg.position[1], 2 * np.pi)
        self.get_logger().debug(f" Pole angle [rad]: {self.pole_angle}")

    def pub_cart_force(self):
        u = self.controller.compute_control()

        print(u)

        joint_state_msg = JointState()

        # TODO: fill the rest of the fields
        # joint_state_msg.name.extend([["cartJoint"]])
        # joint_state_msg.effort.extend([u])

        # self.publisher.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    cartpole_node = CartpoleNode()
    rclpy.spin(cartpole_node)
    cartpole_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
