import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node, ParameterDescriptor, ParameterType
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist


class BalancioPID(Node):
    def __init__(self):
        super().__init__("balancio_pid")
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.declare_parameter(
            name="set_point",
            descriptor=ParameterDescriptor(
                description="Controller set-point", type=ParameterType.PARAMETER_DOUBLE
            ),
            value=0.0,
        )
        self.declare_parameter(
            name="kp",
            descriptor=ParameterDescriptor(
                description="Proportional gain", type=ParameterType.PARAMETER_DOUBLE
            ),
        )
        self.declare_parameter(
            name="ki",
            descriptor=ParameterDescriptor(
                description="Integral gain", type=ParameterType.PARAMETER_DOUBLE
            ),
        )
        self.declare_parameter(
            name="kd",
            descriptor=ParameterDescriptor(
                description="Derivative gain", type=ParameterType.PARAMETER_DOUBLE
            ),
        )
        self.declare_parameter(
            name="sum_constraint",
            descriptor=ParameterDescriptor(
                description="Integral error constraint",
                type=ParameterType.PARAMETER_DOUBLE,
            ),
            value=1.0,
        )

        self.dt = 0.05
        self.subscription = self.create_subscription(Imu, "/imu", self.imu_callback, 1)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(self.dt, self.update)

        self.theta = 0
        self.set_point = 0
        self.error_sum = 0
        self.error_prev = 0

        self.set_point = (
            self.get_parameter("set_point").get_parameter_value().double_value
        )
        self.kp = self.get_parameter("kp").get_parameter_value().double_value
        self.ki = self.get_parameter("ki").get_parameter_value().double_value
        self.kd = self.get_parameter("kd").get_parameter_value().double_value
        self.sum_constraint = (
            self.get_parameter("sum_constraint").get_parameter_value().double_value
        )

    def imu_callback(self, msg: Imu):
        self.get_logger().debug("Received IMU data:")

        rotation_matrix = R.from_quat(
            np.array(
                [
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                ]
            )
        )

        self.theta = np.arcsin(-rotation_matrix.as_matrix()[2, 0])

        self.get_logger().debug(
            f" Pitch Angle: {self.theta} rad. - {np.degrees(self.theta)} deg."
        )

    def update(self):
        error = self.theta - self.set_point

        self.get_logger().info(f"Error: {error}")

        self.error_sum += error
        self.error_sum = np.clip(
            self.error_sum, -self.sum_constraint, self.sum_constraint
        )

        u = (
            (
                self.kp * error
                + self.ki * self.error_sum * self.dt
                + self.kd * (error - self.error_prev) / self.dt
            )
            if np.linalg.norm(self.theta) < np.pi / 4
            else 0.0
        )

        self.get_logger().debug(f"Control input: {u}")

        self.error_prev = error

        twist_msg = Twist()
        twist_msg.linear.x = u

        self.publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    balancio_pid = BalancioPID()
    rclpy.spin(balancio_pid)
    balancio_pid.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
