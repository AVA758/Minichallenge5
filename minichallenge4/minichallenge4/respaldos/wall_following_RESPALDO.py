import rclpy  
from rclpy.node import Node  
from geometry_msgs.msg import Twist  
from sensor_msgs.msg import LaserScan  
import numpy as np  
from copy import deepcopy 
import signal # To handle Ctrl+C 
import sys # To exit the program 

  

class FollowWalls(Node):  

    def __init__(self):  
        super().__init__('wall_following')  

        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  

         # Handle shutdown gracefully (stop the robot) 
        signal.signal(signal.SIGINT, self.shutdown_function) 
        self.sub = self.create_subscription(LaserScan, "scan", self.lidar_cb, 10)  
        self.lidar = LaserScan() # Data from lidar will be stored here.  
        self.robot_vel = Twist() #Robot velocity  
        self.start_fw_distance = 0.5 # Distance to start the wall following behavior 
        self.fw_distance = 0.2 # Distance to the wall we want to follow 
        self.v_max = 0.4 # Maximum linear velocity of the robot 
        self.w_max = 1.5 # Maximum angular velocity of the robot 
        self.kw2 = 3.0 # Gain for the proximity controller 
        self.kw1 = 3.0 # Gain for the orientation controller
 
        timer_period = 0.1  # seconds 

        # Create a timer to call the timer_callback function at a regular interval  
        self.timer = self.create_timer(timer_period, self.timer_callback) 

        self.get_logger().info("Following Walls Node initialized!!!")  

  

    def timer_callback(self):  

        if self.lidar.ranges: #if we have data inside the lidar message  
            #### ADD YOUR CODE HERE #### 
            # Orientation controller 
            # Get the closest object detected by the lidar 
            closest_range, closest_angle = self.get_closest_object() 
            # Get the angle of a vector that points away from the closest object 
            ao_angle = self.get_ao_angle(closest_angle) 


            if closest_range >  self.start_fw_distance : # If the closest object is far away, we can move forward 
                self.robot_vel.linear.x = self.v_max 
                self.robot_vel.angular.z = 0.0 
                self.get_logger().info("No obstacle...")  

            else: # If the closest object is close, we start following the wall.  

                # Get the  fwcc angle to the closest object 
                fwcc_angle = self.get_fwcc_angle(ao_angle) 
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


            self.pub_cmd_vel.publish(self.robot_vel)  

  

    def lidar_cb(self, msg):  

        ## This function receives the ROS LaserScan message  
        self.lidar =  msg 

        # Clean inf values from the lidar data 
        inf_value = 100.0 # Set the inf values to a very large value 
        for i in range(len(self.lidar.ranges)): 
            if np.isinf(self.lidar.ranges[i]): 
                self.lidar.ranges[i] = inf_value 

         

    def get_regions(self, lidar): 

        # This function takes as input the lidar data (LaserScan) and returns the distance to the closest object in each region 
        # The regions are defined as follows: 
        # front:  - 2pi/20 to 2pi/20 rad 
        # front_left:  2pi/20 to 6pi/20 rad 
        # left:  6pi/20 to 10pi/20 rad 
        # back_left:  10pi/20 to 14pi/20 rad 
        size = len(lidar.ranges) # number of elements in the lidar data 
        regions = { 
            'front':  min(lidar.ranges[int(19*size/20):int(20*size/20-1)]+lidar.ranges[0:int(size/20)]), 
            'front_left':  min(lidar.ranges[int(size/20):int(3*size/20)]), 
            'left': min(lidar.ranges[int(3*size/20):int(5*size/20)]), 
            'back_left':  min(lidar.ranges[int(5*size/20):int(7*size/20)]), 
            'front_right':  min(lidar.ranges[int(17*size/20):int(19*size/20)]), 
            'right': min(lidar.ranges[int(15*size/20):int(17*size/20)]), 
            'back_right':  min(lidar.ranges[int(13*size/20):int(15*size/20)]) 
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
        self.pub_cmd_vel.publish(stop_twist) # Send stop command to the robot 
        rclpy.shutdown() # Shutdown the node 
        sys.exit(0) # Exit the program 

     

     

def main(args=None):  
    rclpy.init(args=args)  
    m_p=FollowWalls()  
    rclpy.spin(m_p)  
    m_p.destroy_node()  
    rclpy.shutdown()  

   

if __name__ == '__main__':  
    main()  