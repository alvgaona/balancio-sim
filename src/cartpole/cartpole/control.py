import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .controllers.pid import CartpolePID
from .controllers.controller import Controller


class CartpoleNode(Node):
    def __init__(self, dt: float, controller: Controller, model=None):
        super().__init__("cartpole_node")
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.dt = dt
        self.model = model
        self.controller = controller

        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.joint_states_cb, 1
        )
        self.publisher = self.create_publisher(JointState, "/joint_command", 10)
        self.timer = self.create_timer(self.dt, self.pub_cart_force)

        self.pole_angle = 0.0

    def joint_states_cb(self, msg: JointState) -> None:
        self.pole_angle = msg.position[1]

    def pub_cart_force(self):
        error = self.pole_angle - self.controller.set_point

        self.get_logger().debug(
            f"Pole angle [rad]: {self.pole_angle}, Error [rad]: {error}"
        )

        u = self.controller.compute_control(error)

        self.get_logger().debug(f" Control input: {u}")

        joint_state_msg = JointState()

        joint_state_msg.name.extend(["cartJoint"])
        joint_state_msg.effort.append(u)

        self.publisher.publish(joint_state_msg)


def main(args=None):
    dt = 0.025

    pid = CartpolePID(
        dt=dt,
        kp=10.0,
        ki=0.0,
        kd=5.0,
        set_point=0.0,
        sum_constraint=(-1.0, 1.0),
    )

    rclpy.init(args=args)
    cartpole_node = CartpoleNode(dt=dt, controller=pid)
    rclpy.spin(cartpole_node)
    cartpole_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
