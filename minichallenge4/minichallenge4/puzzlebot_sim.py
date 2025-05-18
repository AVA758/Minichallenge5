import minichallenge4.utils.puzzlebot_kinematics as puzzlebot_kinematics
import numpy as np
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class PuzzlebotSim(Node):
    def __init__(self):
        super().__init__('puzzlebot_sim')

        # Robot parameters declaration
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        # Get the parameters
        r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        l = self.get_parameter('wheel_base').get_parameter_value().double_value

        # Subscribers
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Publishers
        self.right_wheel_speed_publisher = self.create_publisher(Float32, 'wr', 10)
        self.left_wheel_speed_publisher = self.create_publisher(Float32, 'wl', 10)
        
        # Node variables
        self.inverse_puzzlebot_kinematic_model = puzzlebot_kinematics.get_inverse_puzzlebot_kinematic_model(r, l)

        # Log the node start
        self.get_logger().info('Puzzlebot simulation node started.')

    def cmd_vel_callback(self, msg):
        # Update the robot speeds
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Obtain the wheel speeds based on the inverse kinematic model
        wheels_speeds = self.inverse_puzzlebot_kinematic_model @ np.array([linear_speed, angular_speed])

        # Float32 messages to pubish wheel speeds
        wr_msg = Float32()
        wl_msg = Float32()
        wr_msg.data = wheels_speeds[0]
        wl_msg.data = wheels_speeds[1]

        # Publish the wheel speeds
        self.right_wheel_speed_publisher.publish(wr_msg)
        self.left_wheel_speed_publisher.publish(wl_msg)

def main():
    rclpy.init()
    node = PuzzlebotSim()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().info('Node interrupted. Shutting down...')
        node.get_logger().info(f'Error: {e}')
        if rclpy.ok():
            rclpy.shutdown()
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
