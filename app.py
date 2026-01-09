import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import atan2, sqrt, pow, cos, sin, asin
import threading
import sys

# CONSTANTS FOR CONVERSION
MAX_LINEAR_SPEED = 0.22  # m/s (TurtleBot3 limit)
MAX_PWM = 2000
MIN_PWM = 1000
STOP_PWM = 1500

class RoverController(Node):
    def __init__(self):
        super().__init__('rover_controller')
        
        # Publisher to move the robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber to know where the robot is
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Robot State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.target_x = None
        self.target_y = None
        self.moving = False
        
        # Control Loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        print("Rover Controller Initialized.")
        print(f"PWM Range: {MIN_PWM} (Back) <-> {STOP_PWM} (Stop) <-> {MAX_PWM} (Fwd)")

    def odom_callback(self, msg):
        # Update Position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Convert Quaternion to Euler (Yaw)
        q = msg.pose.pose.orientation
        # Standard calculation to get yaw from quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = atan2(siny_cosp, cosy_cosp)

    def get_pwm(self, speed_mps):
        """
        Maps speed (m/s) to your 1000-2000 PWM range.
        """
        # Clamp speed to max limits
        if speed_mps > MAX_LINEAR_SPEED: speed_mps = MAX_LINEAR_SPEED
        if speed_mps < -MAX_LINEAR_SPEED: speed_mps = -MAX_LINEAR_SPEED
        
        # Calculate ratio (-1.0 to 1.0)
        ratio = speed_mps / MAX_LINEAR_SPEED
        
        # Map to PWM (500 is the range from 1500 to 2000)
        pwm_val = 1500 + (ratio * 500)
        return int(pwm_val)

    def control_loop(self):
        if not self.moving or self.target_x is None:
            return

        # 1. Calculate distance to goal
        dist_x = self.target_x - self.x
        dist_y = self.target_y - self.y
        distance = sqrt(pow(dist_x, 2) + pow(dist_y, 2))
        
        # 2. Calculate angle to goal
        goal_angle = atan2(dist_y, dist_x)
        heading_error = goal_angle - self.yaw
        
        # Normalize angle to [-pi, pi]
        while heading_error > 3.14159: heading_error -= 2 * 3.14159
        while heading_error < -3.14159: heading_error += 2 * 3.14159

        cmd = Twist()

        # STOP Condition: Close enough
        if distance < 0.05:
            print(f"\nTarget Reached! ({self.x:.2f}, {self.y:.2f})")
            self.cmd_vel_pub.publish(Twist()) # Send stop
            self.moving = False
            self.target_x = None
            return

        # LOGIC: P-Controller
        # If looking away from target, turn in place
        if abs(heading_error) > 0.2:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 * heading_error  # Turn speed
        else:
            # If facing target, drive forward and adjust slightly
            cmd.linear.x = 0.2 * distance        # Drive speed
            cmd.angular.z = 0.3 * heading_error  # Adjustment speed

        # Cap speeds for safety
        if cmd.linear.x > 0.2: cmd.linear.x = 0.2
        if cmd.angular.z > 0.5: cmd.angular.z = 0.5

        # --- CONVERT TO YOUR PWM FORMAT (Differential Drive) ---
        # Simple differential steering math
        # Left = linear - angular, Right = linear + angular
        # (Simplified, actual physics depends on wheel base width)
        left_motor_speed = cmd.linear.x - (cmd.angular.z * 0.1) 
        right_motor_speed = cmd.linear.x + (cmd.angular.z * 0.1)
        
        left_pwm = self.get_pwm(left_motor_speed)
        right_pwm = self.get_pwm(right_motor_speed)

        # Print current status nicely
        sys.stdout.write(f"\rDist: {distance:.2f}m | PWM L: {left_pwm} R: {right_pwm} | Angle Err: {heading_error:.2f}")
        sys.stdout.flush()

        # Send command to ROS simulation
        self.cmd_vel_pub.publish(cmd)

    def input_thread(self):
        while rclpy.ok():
            if not self.moving:
                try:
                    print("\n--- NEW COMMAND ---")
                    x_in = float(input("Enter Target X: "))
                    y_in = float(input("Enter Target Y: "))
                    
                    self.target_x = x_in
                    self.target_y = y_in
                    self.moving = True
                    print(f"Moving to ({x_in}, {y_in})...")
                except ValueError:
                    print("Invalid number. Try again.")

def main(args=None):
    rclpy.init(args=args)
    rover = RoverController()
    
    # Run input in a separate thread so it doesn't block the ROS loop
    thread = threading.Thread(target=rover.input_thread)
    thread.daemon = True
    thread.start()
    
    rclpy.spin(rover)
    rover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
