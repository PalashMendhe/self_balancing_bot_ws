import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        self.x = 0.0
        self.x_dot = 0.0
        self.theta = 0.0
        self.theta_dot = 0.0
        self.az = 9.8
        self.roll = 0.0
        self.start_time = None
        self.x_integral = 0.0
        self.theta = 0.0
        self.theta_dot = 0.0
        self.last_imu_time = None
        self.x_dot_filtered = 0.0
        self.theta_filtered = 0.0

        # Inner loop (theta)
        self.kp_theta = 20.00
        self.kd_theta = 4.00

        # Outer loop (x) - disabled until inner loop works
        self.kp_x = 0.0
        self.kd_x = 0.0
        self.ki_x = 0.0

        self.get_logger().info('Balance controller started (PID mode)')

        self.imu_subscriber_ = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        self.odom_subscriber_ = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.02, self.control_loop)

    def quaternion_to_euler(self, x, y, z, w):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def imu_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.last_imu_time is None:
            self.last_imu_time = now
            return

        dt = now - self.last_imu_time
        self.last_imu_time = now

        if dt <= 0 or dt > 0.1:
            return

        gyro_y = msg.angular_velocity.y
        ax = msg.linear_acceleration.x
        az = msg.linear_acceleration.z

        # Only use accel correction when not in heavy acceleration
        accel_magnitude = np.sqrt(ax**2 + az**2)
        if 8.0 < accel_magnitude < 12.0:  # near 1g, reliable
            accel_theta = np.arctan2(ax, az)
            alpha = 0.98
            self.theta = alpha * (self.theta + gyro_y * dt) + (1 - alpha) * accel_theta
        else:
            self.theta += gyro_y * dt

        self.theta_dot = gyro_y
        self.az = az

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        raw_x_dot = msg.twist.twist.linear.x
        
        self.x_dot_filtered = 0.8 * self.x_dot_filtered + 0.2 * raw_x_dot
        self.x_dot = self.x_dot_filtered

    def control_loop(self):
        if self.start_time is None:
            self.start_time = self.get_clock().now()
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < 2.0:
            self.theta = 0.0
            return

        # Fall detection

        if abs(self.theta) > 0.8:
            msg = Twist()
            self.cmd_vel_publisher_.publish(msg)
            self.get_logger().warn(f'FALLEN: theta={self.theta:.3f}')
            self.theta = 0.0
            return

        dt = 0.02
        x_error = 0.0 - self.x
        self.x_integral += x_error * dt
        self.x_integral = np.clip(self.x_integral, -0.3, 0.3)

        theta_setpoint = (
            self.kp_x * x_error +
            self.kd_x * (0.0 - self.x_dot) +
            self.ki_x * self.x_integral
        )
        theta_setpoint = np.clip(theta_setpoint, -0.08, 0.08)
        self.theta_filtered = 0.9 * self.theta_filtered + 0.2 * theta_setpoint
        theta_for_control = self.theta_filtered
        theta_error = theta_for_control - self.theta
        u = -(self.kp_theta * theta_error - self.kd_theta * self.theta_dot)
        u = np.clip(u, -8.0, 8.0)

        msg = Twist()
        msg.linear.x = u
        self.cmd_vel_publisher_.publish(msg)

        self.get_logger().info(
            f'theta: {self.theta:.3f}, theta_sp: {theta_setpoint:.3f}, '
            f'u: {u:.3f}, x: {self.x:.3f}, x_dot: {self.x_dot:.3f}, '
            f'elapsed: {elapsed:.1f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()