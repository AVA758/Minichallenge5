import numpy as np
import rclpy
import transforms3d as t3d

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool

class PuzzlebotController(Node):
    def __init__(self):
        super().__init__('puzzlebot_controller')

        # Declare parameters
        self.declare_parameter('controller_update_rate', 25.0)
        self.declare_parameter('distance_tolerance', 0.1)
        self.declare_parameter('angular_tolerance', 0.1)
        self.declare_parameter('v_Kp', 0.5)
        self.declare_parameter('alpha_Kp', 0.5)
        self.declare_parameter('beta_Kp', 0.5)
        self.declare_parameter('v_max', 1.0)
        self.declare_parameter('w_max', 3.14)

        # Subscribers
        self.create_subscription(Pose2D, 'setpoint', self.setpoint_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.next_point_publisher = self.create_publisher(Bool, 'next_point', 10)

        # Node variables
        self.controller_timer = None
        self.update_rate = self.get_parameter('controller_update_rate').get_parameter_value().double_value
        self.distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self.angular_tolerance = self.get_parameter('angular_tolerance').get_parameter_value().double_value
        self.v_Kp = self.get_parameter('v_Kp').get_parameter_value().double_value
        self.alpha_Kp = self.get_parameter('alpha_Kp').get_parameter_value().double_value
        self.beta_Kp = self.get_parameter('beta_Kp').get_parameter_value().double_value
        self.v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self.w_max = self.get_parameter('w_max').get_parameter_value().double_value
        self.robot_pose = Pose2D()
        self.robot_setpoint = Pose2D()

        # Log the node start
        self.get_logger().info('Puzzlebot Controller Node has been started.')

    def setpoint_callback(self, msg):
        # If the controller timer is running, cancel it
        if self.controller_timer is not None:
            self.controller_timer.cancel()

        # Start the controller timer with the new update rate   
        self.controller_timer = self.create_timer(1.0/self.update_rate, self.controller_callback)
        # Update the robot pose with the new setpoint
        self.robot_setpoint = msg

    def odom_callback(self, msg):
        # Extract [x, y] position from the odometry message
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y
        # Extract quaternion orientation from the odometry message
        q = msg.pose.pose.orientation
        # Convert quaternion to Euler angles and update the robot pose
        _, _, self.robot_pose.theta = t3d.euler.quat2euler([q.w, q.x, q.y, q.z])

    def controller_callback(self):
        # Calculate the distance to the setpoint
        distance_to_target = self._get_euclidian_distance_between_poses(self.robot_pose, self.robot_setpoint)
        # Calculate the angle between the robot and the setpoint
        angle_between_poses = np.arctan2(self.robot_setpoint.y - self.robot_pose.y, self.robot_setpoint.x - self.robot_pose.x)
        
        # Calculate the angle to face the target and the setpoint error
        theta_to_face_target_error = angle_between_poses - self.robot_pose.theta
        # Normalize the angle to be within [-pi, pi]
        theta_to_face_target_error = np.arctan2(np.sin(theta_to_face_target_error), np.cos(theta_to_face_target_error))
        # Calculate the theta setpoint error
        theta_setpoint_error = self.robot_setpoint.theta - angle_between_poses 
        # Normalize the angle to be within [-pi, pi]
        theta_setpoint_error = np.arctan2(np.sin(theta_setpoint_error), np.cos(theta_setpoint_error))
        
        # Create a Twist message to publish the robot velocity
        twist_msg = Twist()
        if distance_to_target < self.distance_tolerance and abs(self.robot_setpoint.theta-self.robot_pose.theta) < self.angular_tolerance:
            self.next_point_publisher.publish(Bool(data=True))
            self.controller_timer.cancel()
        else:
            # Perform orientation control on the move
            linear_speed = self.v_Kp * distance_to_target
            angular_speed = self.alpha_Kp * theta_to_face_target_error + self.beta_Kp * theta_setpoint_error
            # If the robot is close to the target, use an only orientation control based on the setpoint angle
            if distance_to_target < self.distance_tolerance:
                # Use only orientation control
                linear_speed = 0
                # Calulate the angle error
                theta_error = self.robot_setpoint.theta - self.robot_pose.theta
                # Normalize the angle to be within [-pi, pi]
                theta_error = np.arctan2(np.sin(theta_error), np.cos(theta_error))
                # Calculate the angular speed as a P controller
                angular_speed = -self.beta_Kp  * theta_error
            # Constrain speeds
            linear_speed = np.clip(linear_speed, -self.v_max, self.v_max)
            angular_speed = np.clip(angular_speed, -self.w_max, self.w_max)
            # Update the Twist message with the calculated velocities
            twist_msg.linear.x = linear_speed
            twist_msg.angular.z = angular_speed

        # Publish the Twist message
        self.cmd_vel_publisher.publish(twist_msg)

    def _get_euclidian_distance_between_poses(self, pose1, pose2):
        # Calculate the Euclidean distance between two poses with sign
        return np.sqrt((pose1.x - pose2.x) ** 2 + (pose1.y - pose2.y) ** 2)

def main():
    rclpy.init()
    node = PuzzlebotController()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().info('Node interrupted. Shutting down...')
        node.get_logger().error(f'Error: {e}')
        if rclpy.ok():
            rclpy.shutdown()
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()