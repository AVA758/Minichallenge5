import numpy as np
import rclpy
import transforms3d as t3d

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool

from sensor_msgs.msg import LaserScan
from copy import deepcopy 
import signal # To handle Ctrl+C 
import sys # To exit the program 

class PuzzlebotController(Node):
    def __init__(self):
        super().__init__('puzzlebot_controller')

        self.mode = "go2goal"


        # Declare parameters of go2goal
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





        # Wall following parameters
        signal.signal(signal.SIGINT, self.shutdown_function) 
        self.sub = self.create_subscription(LaserScan, "scan", self.lidar_cb, 10)  
        self.lidar = LaserScan() # Data from lidar will be stored here.  
        self.robot_vel = Twist() #Robot velocity  
        self.start_fw_distance = 0.5 # Distance to start the wall following behavior 
        self.fw_distance = 0.2 # Distance to the wall we want to follow 
        self.kw2 = 3.0 # Gain for the proximity controller 
        self.kw1 = 3.0 # Gain for the orientation controller
 
        timer_period = 0.1  # seconds 

        # Create a timer to call the timer_callback function at a regular interval  
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        self.get_logger().info("Following Walls Node has also been started!!!")  





    def setpoint_callback(self, msg):
        # If the controller timer is running, cancel it
        if self.controller_timer is not None:
            self.controller_timer.cancel()

        # Start the controller timer with the new update rate   
        self.controller_timer = self.create_timer(1.0/self.update_rate, self.controller_callback)
        # Update the robot pose with the new setpoint
        self.robot_setpoint = msg


        self.get_logger().info('Setpoint_callback completed')





    def odom_callback(self, msg):
        # Extract [x, y] position from the odometry message
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y
        # Extract quaternion orientation from the odometry message
        q = msg.pose.pose.orientation
        # Convert quaternion to Euler angles and update the robot pose
        _, _, self.robot_pose.theta = t3d.euler.quat2euler([q.w, q.x, q.y, q.z])
        #self.get_logger().info('odom_callback completed')





    def _get_euclidian_distance_between_poses(self, pose1, pose2):
        # Calculate the Euclidean distance between two poses with sign
        return np.sqrt((pose1.x - pose2.x) ** 2 + (pose1.y - pose2.y) ** 2)




    def lidar_cb(self, msg):  
        ## This function receives the ROS LaserScan message  
        self.lidar =  msg

        # Clean inf values from the lidar data 
        inf_value = 100.0 # Set the inf values to a very large value 
        for i in range(len(self.lidar.ranges)): 
            if np.isinf(self.lidar.ranges[i]): 
                self.lidar.ranges[i] = inf_value 
        #self.get_logger().info('lidar_cb completed')

        



    def timer_callback(self):  
        if self.lidar.ranges: #if we have data inside the lidar message  
            #### ADD YOUR CODE HERE #### 
            # Orientation controller 
            # Get the closest object detected by the lidar 
            closest_range, closest_angle = self.get_closest_object() 
            # Get the angle of a vector that points away from the closest object 
            self.ao_angle = self.get_ao_angle(closest_angle) 


            if self.mode=="EvadingObstacle" : # If the closest object is far away, we can move forward 
                if self.is_path_to_goal_clear():
                    self.mode = "go2goal"
                    self.get_logger().info("No obstacle detected. Continuing to goal...")  


            elif self.mode == "go2goal" and closest_range <= self.start_fw_distance :  # If the closest object is close, we start following the wall.
                self.mode = "EvadingObstacle"
                self.get_logger().info("Obstacle detected. Evading...")





    def controller_callback(self):
        if self.mode == "EvadingObstacle":
            self.evadeObstacle_callback()

        elif self.mode == "go2goal":
            self.go2goal_callback()





    #NORMAL PROCEDURE
    def go2goal_callback(self):
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
        self.get_logger().info("Cmd_vel sent!")  
        self.cmd_vel_publisher.publish(twist_msg)





    # WALL FOLLOWER    
    def evadeObstacle_callback(self):
        # Get the  fwcc angle to the closest object 
        fwcc_angle = self.get_fwcc_angle(self.ao_angle) 
        wfw1 = self.kw1 * (fwcc_angle) # Proportional controller to align the robot with the wall 

        # Get the closest object to the left  
        size = len(self.lidar.ranges) 
        left_distance = min(self.lidar.ranges[int(3*size/20):int(5*size/20)]) 
        proximity_controller_error = left_distance - self.fw_distance  
        wfw2 = self.kw2 * proximity_controller_error 

        # Get distance to the front  
        front_region = self.lidar.ranges[int(19*size/20):int(20*size/20)]+ self.lidar.ranges[int(0):int(1*size/20)] 
        front_distance = min(front_region) 

        # Get distance to other regions (front_left, back_left) 
        front_left_distance = min(self.lidar.ranges[int(1*size/20):int(3*size/20)]) 
        back_left_distance = min(self.lidar.ranges[int(5*size/20):int(7*size/20)]) 

        # Look for inner and outer corners 
        if front_distance < self.fw_distance: # Check if there is an obstacle in front of the robot.  

            # Just turn around to the right (if we are following the wall counter-clockwise)  
            self.get_logger().warn("Inner corner detected, turn right") 
            self.robot_vel.linear.x = 0.0 
            self.robot_vel.angular.z = -self.w_max

        elif front_distance < self.fw_distance*2.0:  # Check if there is an obstacle close to the front of the robot. 
            # Slow down a little  
            self.get_logger().warn("Inner corner detected, slow_down eft") 
            self.robot_vel.linear.x = 0.5 * self.v_max 

        # Check if there is an outer corner 

        elif back_left_distance < self.start_fw_distance and left_distance >= self.start_fw_distance and front_left_distance >= self.start_fw_distance and front_distance >= self.start_fw_distance: 
            self.get_logger().warn("Outer corner detected, turn left") 

            # Move forward_slowly and turn left 
            self.robot_vel.linear.x = self.v_max/5.0 
            self.robot_vel.angular.z = self.w_max 

        else: # Normal case, follow the wall 
            # Here we are following the wall counter-clockwise (wall on the left) 
            self.robot_vel.linear.x = self.v_max 
            self.robot_vel.angular.z = wfw1 + wfw2 


        self.cmd_vel_publisher.publish(self.robot_vel)  



   

        #BUG0 ALGORITHM
    def is_path_to_goal_clear(self):
        # Calculate angle to goal
        # Previius self.ao_angle = self.get_ao_angle(closest_angle) can't be used because that one is based on the lidar
        # in escense, it's a different angle
        angle_to_goal = np.arctan2(self.robot_setpoint.y - self.robot_pose.y,
                                self.robot_setpoint.x - self.robot_pose.x)
        angle_to_goal = np.arctan2(np.sin(angle_to_goal), np.cos(angle_to_goal))

        distance_to_goal = self._get_euclidian_distance_between_poses(self.robot_pose, self.robot_setpoint)


        #The angle range for when to detect if there is an obstacle in the goal trajectory is a window of 15° in radians
        window= np.deg2rad(15)  # 15 degrees in radians

       
        #Finds all lidar angles
        lidar_angles = [self.lidar.angle_min + i*self.lidar.angle_increment for i in range(len(self.lidar.ranges))]

        # Find ranges within the sector
        # From all the lidar angles, it only saves the readings from the previously stated window
        ranges_in_sector = []
        for i, angle in enumerate(lidar_angles):
            angle = np.arctan2(np.sin(angle), np.cos(angle))  # wrap angle
            if abs(angle - angle_to_goal) <= window:
                ranges_in_sector.append(self.lidar.ranges[i])

        # If any obstacle is closer than the goal distance in this sector, then the path is not clear
        if len(ranges_in_sector) == 0:
            return True  # no data, assume clear

        if min(ranges_in_sector) < distance_to_goal:
            return False
        else:
            return True





    def get_regions(self): 
        # This function takes as input the lidar data (LaserScan) and returns the distance to the closest object in each region 
        # The regions are defined as follows: 
        # front:  - 2pi/20 to 2pi/20 rad 
        # front_left:  2pi/20 to 6pi/20 rad 
        # left:  6pi/20 to 10pi/20 rad 
        # back_left:  10pi/20 to 14pi/20 rad 
        size = len(self.lidar.ranges) # number of elements in the lidar data 
        regions = { 
            'front':  min(self.lidar.ranges[int(19*size/20):int(20*size/20-1)]+self.lidar.ranges[0:int(size/20)]), 
            'front_left':  min(self.lidar.ranges[int(size/20):int(3*size/20)]), 
            'left': min(self.lidar.ranges[int(3*size/20):int(5*size/20)]), 
            'back_left':  min(self.lidar.ranges[int(5*size/20):int(7*size/20)]), 
            'front_right':  min(self.lidar.ranges[int(17*size/20):int(19*size/20)]), 
            'right': min(self.lidar.ranges[int(15*size/20):int(17*size/20)]), 
            'back_right':  min(self.lidar.ranges[int(13*size/20):int(15*size/20)]) 
        } 
        return regions 

     



    def get_fwcc_angle(self, closest_angle): 

        # This function takes as input the angle to the closest object detected in any direction. 
        fwcc_angle = closest_angle + np.pi/2 
        # wrap the angle to [-pi,pi] 
        fwcc_angle = np.arctan2(np.sin(fwcc_angle), np.cos(fwcc_angle)) 
        return fwcc_angle 
             




    def get_closest_object(self): 

        # This function uses the self.lidar data and returns  
        # the range and angle to the closest object detected by the lidar. 
        # remove inf values from the lidar data 
        closest_range = min(self.lidar.ranges) 
        closest_index = self.lidar.ranges.index(closest_range) 
        closest_angle = self.lidar.angle_min + closest_index*self.lidar.angle_increment 

        # wrap the angle to [-pi,pi] 
        closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle)) 
        return closest_range, closest_angle 

     



    def get_ao_angle(self, closest_angle): 

        #This function takes as input the angle to the closest object detected 
        # by the lidar and returns the angle of a vector that points away from it.  
        ao_angle = closest_angle + np.pi  

        # wrap the angle to [-pi,pi] 
        ao_angle = np.arctan2(np.sin(ao_angle), np.cos(ao_angle)) 
        return ao_angle 

     

     

    def shutdown_function(self, signum, frame): 

        # This function is called when the program receives a SIGINT signal (Ctrl+C) 
        # It stops the robot and exits the program gracefully. 
        self.get_logger().info("Ctrl+C pressed. Stopping robot...") 
        stop_twist = Twist()  # All zeros to stop the robot 
        self.cmd_vel_publisher.publish(stop_twist) # Send stop command to the robot 
        rclpy.shutdown() # Shutdown the node 
        sys.exit(0) # Exit the program 








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