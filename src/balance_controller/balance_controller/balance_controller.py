import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
from scipy.signal import place_poles

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')
        self.x = 0.0
        self.x_dot = 0.0
        self.theta = 0.0
        self.theta_dot = 0.0
        self.start_time = None

        M, m, l, g = 0.8, 0.2, 0.235, 9.81

        A = np.array([
            [0, 1, 0, 0],
            [0, 0, -(m*g)/M, 0],
            [0, 0, 0, 1],
            [0, 0, (M+m)*g/(M*l), 0]
        ])
        B = np.array([[0], [1/M], [0], [-1/(M*l)]])

        desired_poles = np.array([-2+0.5j, -2-0.5j, -3+1j, -3-1j])
        result = place_poles(A, B, desired_poles)
        self.K = result.gain_matrix
        self.get_logger().info(f'Gain matrix K: {self.K}')
        self.imu_subscriber_ = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.odom_subscriber_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.02, self.control_loop)

    def quaternion_to_euler(self, x, y, z, w):
        # Roll (x-axis)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis) -- this is your theta
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis) -- not needed but complete for reference
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
            
    def imu_callback(self, msg):
        q = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.theta = pitch
        self.theta_dot = msg.angular_velocity.y
        self.az = msg.linear_acceleration.z
        self.roll = roll

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.x_dot = msg.twist.twist.linear.x
    def control_loop(self):
        if self.start_time is None:
            self.start_time = self.get_clock().now()
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < 2.0:  # 2 second warmup
            return
        if abs(self.theta) > 0.5:
            return
        if abs(self.az) < 5.0:
            msg = Twist()
            self.cmd_vel_publisher_.publish(msg)
            return
        state = np.array([self.x * 0.07, self.x_dot * 0.07, self.theta, self.theta_dot])
        u = float(-self.K @ state)
        u = np.clip(u, -3.0, 3.0)
        msg = Twist()
        msg.linear.x = u
        msg.angular.z = -self.roll * 3.0
        
        self.cmd_vel_publisher_.publish(msg)
        
        self.get_logger().info(f'theta: {self.theta:.3f}, elapsed: {elapsed:.3f}, control: {u:.3f}, theta_dot: {self.theta_dot:.3f}, x: {self.x:.3f}, x_dot: {self.x_dot:.3f}, az: {self.az:.3f}, roll: {self.roll:.3f}')
        

    
def main(args=None):
    rclpy.init(args=args)
    balance_controller = BalanceController()
    rclpy.spin(balance_controller)
    balance_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()