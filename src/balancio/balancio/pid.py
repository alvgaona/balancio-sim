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

        self.dt = 0.05
        self.subscription = self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(self.dt, self.update)

        self.theta = 0
        self.desired_theta = 0
        self.error_sum = 0
        self.error_prev = 0
        self.kp = self.get_parameter("kp").get_parameter_value().double_value
        self.ki = self.get_parameter("ki").get_parameter_value().double_value
        self.kd = self.get_parameter("kd").get_parameter_value().double_value

    def imu_callback(self, msg: Imu):
        self.get_logger().debug("Received IMU data:")

        axis_angle = R.from_quat(
            np.array(
                [
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                ]
            )
        ).as_euler("zxy", degrees=False)

        self.theta = axis_angle[2]

        self.get_logger().debug(f" Euler Angles: {axis_angle}")
        self.get_logger().debug(f" Tilt angle: {np.degrees(self.theta)}")

    def update(self):
        error = self.desired_theta - self.theta
        self.error_sum += error
        self.error_sum = np.clip(self.error_sum, -1, 1)

        u = (
            self.kp * error
            + self.ki * self.error_sum * self.dt
            + self.kd * (error - self.error_prev) / self.dt
        )

        self.error_prev = error

        twist_msg = Twist()
        twist_msg.linear.x = (
            u if self.theta < np.pi / 2 and self.theta > -np.pi / 2 else 0.0
        )

        self.publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = BalancioPID()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
