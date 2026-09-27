from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class BalanceController(Node):

    def __init__(self):
        super().__init__('balance_controller')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        self.declare_parameter('kp_theta', 30.00)
        self.declare_parameter('kd_theta', 5.00)
        self.declare_parameter('ki_theta', 0.05)
        self.declare_parameter('kp_x', 0.00)
        self.declare_parameter('kd_x', 0.00)
        self.declare_parameter('ki_x', 0.00)

        self.kp_theta = self.get_parameter('kp_theta').value
        self.kd_theta = self.get_parameter('kd_theta').value
        self.ki_theta = self.get_parameter('ki_theta').value
        self.kp_x = self.get_parameter('kp_x').value
        self.kd_x = self.get_parameter('kd_x').value
        self.ki_x = self.get_parameter('ki_x').value

        self.x = 0.0
        self.x_dot = 0.0
        self.theta = 0.0
        self.theta_dot = 0.0
        self.start_time = None
        self.last_control_time = None
        self.last_imu_time = None
        self.x_integral = 0.0
        self.theta_integral = 0.0
        self.x_dot_filtered = 0.0
        self.theta_filtered = 0.0
        self.calibrated = False
        self.gyro_bias = 0.0
        self.calibration_samples = []
        self.is_fallen = False

        self.get_logger().info('Balance controller started (PID mode)')

        self.imu_subscriber_ = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        self.odom_subscriber_ = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.02, self.control_loop)

    def imu_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.last_imu_time is None:
            self.last_imu_time = now
            return

        dt = now - self.last_imu_time
        self.last_imu_time = now

        if dt <= 0 or dt > 0.1:
            return

        # Calibration first
        if not self.calibrated:
            self.calibration_samples.append(msg.angular_velocity.y)
            if len(self.calibration_samples) >= 200:
                self.gyro_bias = float(np.mean(self.calibration_samples))
                self.calibrated = True
                self.get_logger().info(f'Gyro bias calibrated: {self.gyro_bias:.5f}')
            return  # don't integrate during calibration

        # Bias-corrected gyro BEFORE integration
        gyro_y = msg.angular_velocity.y - self.gyro_bias
        ax = msg.linear_acceleration.x
        az = msg.linear_acceleration.z

        accel_magnitude = np.sqrt(ax**2 + az**2)
        if 8.0 < accel_magnitude < 12.0:
            accel_theta = np.arctan2(ax, az)
            alpha = 0.85
            self.theta = alpha * (self.theta + gyro_y * dt) + (1.0 - alpha) * accel_theta
        else:
            self.theta += gyro_y * dt

        self.theta_dot = gyro_y

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        raw_x_dot = msg.twist.twist.linear.x

        self.x_dot_filtered = 0.95 * self.x_dot_filtered + 0.05 * raw_x_dot
        self.x_dot = self.x_dot_filtered

    def control_loop(self):
        # Reload parameters dynamically for live calibration
        self.kp_theta = self.get_parameter('kp_theta').value
        self.kd_theta = self.get_parameter('kd_theta').value
        self.ki_theta = self.get_parameter('ki_theta').value
        self.kp_x = self.get_parameter('kp_x').value
        self.kd_x = self.get_parameter('kd_x').value
        self.ki_x = self.get_parameter('ki_x').value

        if self.start_time is None:
            self.start_time = self.get_clock().now()
            self.last_control_time = self.start_time
            return

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        dt = (now - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = now
        if dt <= 0.0 or dt > 0.1:
            dt = 0.02

        if not self.calibrated:
            msg = Twist()
            self.cmd_vel_publisher_.publish(msg)
            return

        # Fall detection with persistent latch
        if abs(self.theta) > 0.8:
            self.is_fallen = True

        if self.is_fallen:
            msg = Twist()
            self.cmd_vel_publisher_.publish(msg)
            self.get_logger().warn(
                f'FALLEN: theta={self.theta:.3f}. Motor output disabled.',
                throttle_duration_sec=2.0
            )
            return

        x_error = 0.0 - self.x
        self.x_integral += x_error * dt
        self.x_integral = float(np.clip(self.x_integral, -0.3, 0.3))

        theta_setpoint = (
            self.kp_x * x_error +
            self.kd_x * (0.0 - self.x_dot) +
            self.ki_x * self.x_integral
        )
        theta_setpoint = float(np.clip(theta_setpoint, -0.08, 0.08))
        self.theta_filtered = 0.9 * self.theta_filtered + 0.1 * theta_setpoint

        theta_for_control = self.theta_filtered
        theta_error = theta_for_control - self.theta
        self.theta_integral += theta_error * dt
        self.theta_integral = float(np.clip(self.theta_integral, -0.3, 0.3))
        u = -(
            self.kp_theta * theta_error -
            self.kd_theta * self.theta_dot +
            self.ki_theta * self.theta_integral
        )
        u = float(np.clip(u, -8.0, 8.0))

        msg = Twist()
        msg.linear.x = u
        self.cmd_vel_publisher_.publish(msg)

        self.get_logger().info(
            f'theta: {self.theta:.3f}, theta_sp: {theta_setpoint:.3f}, '
            f'u: {u:.3f}, x: {self.x:.3f}, x_dot: {self.x_dot:.3f}, '
            f'elapsed: {elapsed:.1f}, gyro_bias: {self.gyro_bias:.5f}',
            throttle_duration_sec=0.2
        )


def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
