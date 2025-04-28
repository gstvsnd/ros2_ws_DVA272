#! /usr/bin/env python3
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
import math
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion

time.sleep(12) # väntar en snabbis innan den kör
start_time = time.time()

class ObstacleDetection(Node):
    """
    Simple obstacle detection node that stops the robot when obstacles are too close.
    Uses a circular detection zone around the robot.
    
    TODO: Implement the detect_obstacle method to avoid obstacles!
    """
    def __init__(self):
        super().__init__("obstacle_detection")
        
        # Safety parameters - use ROS parameter
        self.declare_parameter("stop_distance", 0.35)  # Default if not specified
        self.stop_distance = self.get_parameter("stop_distance").get_parameter_value().double_value
        self.get_logger().info(f"Using stop_distance: {self.stop_distance}m")
        self.pose = Pose()
        self.odom_sub = self.create_subscription(Odometry, "odom", self.get_odom_callback, qos_profile=qos_profile_sensor_data)
        # Store received data
        self.scan_ranges = []
        self.has_scan_received = False
        
        # Default motion command
        self.tele_twist = Twist()
        self.tele_twist.linear.x = 0.0 # max enligt internet är 0.26m/s
        self.tele_twist.angular.z = 0.0 # max enligt min magkänsla är 1.82

        # Hastigheter
        self.speed = 0.15
        self.P = 1.0

        # Goal - Hårdkodat - tas bort vid senare tillfälle
        self.goal_x = 1.9
        self.goal_y = 0.6
        self.goal_reached = False

        # Robotens riktning
        self.yaw = 0.0 # initiering (Problem?)

        # Set up quality of service
        qos = QoSProfile(depth=10)

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", qos)
        
        # Subscribe to laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self.scan_callback, qos_profile=qos_profile_sensor_data
        )

        # Subscribe to teleop commands
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist, "cmd_vel_raw", self.cmd_vel_raw_callback, 
            qos_profile=qos_profile_sensor_data
        )

        # Set up timer for regular checking
        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_odom_callback(self, msg):
        self.pose = msg.pose.pose
        oriList = [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(oriList)
        #self.get_logger().info(f"Robot state  {self.pose.position.x, self.pose.position.y, yaw}")
        self.yaw = yaw # Sparar riktningen

    def scan_callback(self, msg):
        """Store laser scan data when received"""
        self.scan_ranges = msg.ranges
        self.has_scan_received = True

    def cmd_vel_raw_callback(self, msg):
        """Store teleop commands when received"""
        self.tele_twist = msg

    def timer_callback(self):
        """Regular function to check for obstacles"""
        if self.goal_reached == False:
            if self.has_scan_received:
                self.detect_obstacle() # undviker hinder
        else:
            self.get_logger().info(f"(x, y) = ({self.pose.position.x}, {self.pose.position.y})")
            self.get_logger().info(f"Goal reached in {round((time.time() - start_time), 1)} seconds!")
            time.sleep(5)
            self.destroy_node()
        self.cmd_vel_pub.publish(self.tele_twist) # Publicerar till robot

    def detect_obstacle(self):
        """Obstacle Avoidance"""
        # hitta avstånt till främsta hinder
        obstacle_distance = min(self.scan_ranges) 

        if obstacle_distance < self.stop_distance:
            # hitta vinkeln till närmsta hinder
            obstacle_direction = (self.scan_ranges).index(obstacle_distance)
            if obstacle_direction > 180: #
                obstacle_direction -= 360 # normerar

            self.get_logger().info(f"Hinder-mot robot (grader): {obstacle_direction}") # det är mer intuitivt att se än radianer
            obstacle_direction = obstacle_direction * math.pi / 180 # omvandlar till radianer
            theta_gtg = math.atan2((self.goal_x - self.pose.position.x), (self.goal_y - self.pose.position.y))
            theta_obstacle = (obstacle_direction) + self.yaw #summera
            theta_obstacle = (theta_obstacle + math.pi) % (2 * math.pi) - math.pi #normera

            # Bestämm snabbaste omväg & Navigera
            if abs(obstacle_direction) < (math.pi / 2): # Om roboten kollar mot hindret stannar den och riktar up sig.
                self.get_logger().info(f"HINDER")


                if theta_gtg > theta_obstacle:
                    self.get_logger().info(f"STYR VÄNSTER")
                    e_theta = theta_obstacle - self.yaw + (math.pi / 2)
                else:
                    self.get_logger().info(f"STYR HÖGER")
                    e_theta = theta_obstacle - self.yaw - (math.pi / 2)

                e_theta = math.atan2(math.sin(e_theta), math.cos(e_theta)) # ;)
                self.get_logger().info(f"P * e_theta: {self.tele_twist.angular.z}") #Används för att svänga bort från hinder

                # Anpassa fart och riktning:
                self.tele_twist.linear.x = self.speed * abs(math.cos(e_theta)) # [0,1) - bromsar mer ju mer roboten behöver veja
                self.tele_twist.angular.z = self.P * e_theta * 1.1
            else:
                self.go_to_goal() # gå mot målet
        else:
            self.go_to_goal() # gå mot målet

    def go_to_goal(self):
        """Navigating towards Goal"""
        #Avstånd
        goal_distance = math.sqrt((self.goal_x - self.pose.position.x)**2 + (self.goal_y - self.pose.position.y)**2)
        goal_threashhold = 0.1

        #Riktning
        theta_gtg = math.atan2((self.goal_y - self.pose.position.y), (self.goal_x - self.pose.position.x)) #koorddinatsystemet är tvärtom
        e_theta = theta_gtg - self.yaw
        e_theta = math.atan2(math.sin(e_theta), math.cos(e_theta)) # ;)
        
        #Navigering
        #self.get_logger().info(f"Riktning: theta_gtg och self.yaw:  {theta_gtg, self.yaw}") # debugg
        if goal_distance < goal_threashhold:
            self.goal_reached = True
            self.tele_twist.linear.x = 0.0
            self.tele_twist.angular.z = 0.0
        else:
            self.tele_twist.linear.x = self.speed # Fullt ös!
            self.tele_twist.angular.z = self.P * e_theta * 0.9



        """
        TODO: Implement obstacle detection and avoidance!
        
        MAIN TASK:
        - Detect if any obstacle is too close to the robot (closer than self.stop_distance) -ok
        - Turn if obstacle is close -ok
        
        UNDERSTANDING LASER SCAN DATA:
        - self.scan_ranges contains distances to obstacles in meters -ok
        - Each value represents distance at a different angle around the robot -ok
        - Values less than self.stop_distance indicate a close obstacle -ok
        
        UNDERSTANDING POSE (Pose message)
        - self.pose.position.x: x position of the robot -ok
        - self.pose.position.y: y position of the robot -ok
        - yaw: heading of the robot, converted from quaternions for your convinence (in radians) -ok

        CREATE CONTROL SIGNAL FOR ANGULAR VELOCITY
        - Compare angle to goal or obstacle with the current angle of the robot, i.e -ok
        - e_theta = (gtg-yaw) -ok
        - make sure it wraps between pi and -pi
        - e_theta = atan2(sin(e_theta), cos(e_theta)) -ok (köper det)
        - twist.angular.z = P * e_theta
        - Choose P


        CONTROLLING THE ROBOT (Twist message):
        - twist.linear.x: Forward/backward (positive = forward, negative = backward)
        - twist.angular.z: Rotation (positive = left, negative = right)
        - To stop: set twist.linear.x = 0.0 (you can keep angular.z to allow turning)
        """

        # Filter out invalid readings (very small values, infinity, or NaN)
        valid_ranges = [r for r in self.scan_ranges if not math.isinf(r) and not math.isnan(r) and r > 0.01]
        
        # If no valid readings, assume no obstacles
        if not valid_ranges:
            self.cmd_vel_pub.publish(self.tele_twist)
            return
            
        # Find the closest obstacle in any direction (full 360° scan)
        obstacle_distance = min(valid_ranges)

        # TODO: Implement your obstacle detection logic here!
        # Remember to use obstacle_distance and self.stop_distance in your implementation.
        # Remember to find the angle of the closest obstacle

        # For now, just use the teleop command (unsafe - replace with your code)
        twist = self.tele_twist

        # Publish the velocity command
        self.cmd_vel_pub.publish(twist)

    def destroy_node(self):
        """Publish zero velocity when node is destroyed"""
        self.get_logger().info("Shutting down, stopping robot...")
        stop_twist = Twist() # Default Twist has all zeros
        self.cmd_vel_pub.publish(stop_twist)
        super().destroy_node() # Call the parent class's destroy_node


def main(args=None):#
    rclpy.init(args=args)
    obstacle_detection = ObstacleDetection()
    try:
        rclpy.spin(obstacle_detection)
    except KeyboardInterrupt:
        obstacle_detection.get_logger().info('KeyboardInterrupt caught, allowing rclpy to shutdown.')
    finally:
        obstacle_detection.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
