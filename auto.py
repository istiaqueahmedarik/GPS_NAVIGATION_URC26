import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # --- Step 1: Tell the robot where it is (Initial Pose) ---
    #print("--- Set Initial Pose ---")
    #start_x = float(input("Enter current X (usually 0.0): "))
    #start_y = float(input("Enter current Y (usually 0.0): "))
    
    #initial_pose = PoseStamped()
    #initial_pose.header.frame_id = 'odom' # Use 'odom' for non-map navigation
    #initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    #initial_pose.pose.position.x = start_x
    #initial_pose.pose.position.y = start_y
    #initial_pose.pose.orientation.z = 0.0
    #initial_pose.pose.orientation.w = 1.0
    
    #navigator.setInitialPose(initial_pose)
    #navigator.waitUntilNav2Active(localizer=None)

    # --- Step 2: Loop for Goals ---
    while True:
        print("\n--- New Destination ---")
        try:
            goal_x = float(input("Enter Goal X: "))
            goal_y = float(input("Enter Goal Y: "))
        except ValueError:
            print("Invalid number. Try again.")
            continue

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'odom'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.orientation.w = 1.0

        print(f"Going to ({goal_x}, {goal_y})...")
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            # print(f"Distance remaining: {feedback.distance_remaining:.2f} m", end='\r')
            time.sleep(0.1)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("\nGoal Reached!")
        elif result == TaskResult.CANCELED:
            print("\nGoal was canceled!")
        elif result == TaskResult.FAILED:
            print("\nGoal failed!")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
