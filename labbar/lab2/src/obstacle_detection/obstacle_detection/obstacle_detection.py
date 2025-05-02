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

time.sleep(15) # väntar en snabbis innan den kör
start_time = time.time()

class ObstacleDetection(Node):
    """
    Simple obstacle detection node that stops the robot when obstacles are too close.
    Uses a circular detection zone around the robot.
    
    # # # # # # # # # # # # # # # # # # #
    #  Lab2_2                           #
    #  Authors:                         #
    #  Samuel Källstad & Gustav Sand    #
    #                                   #
    # # # # # # # # # # # # # # # # # # #
    
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
        self.speed = 0.25
        self.P = 0.9

        # Goal - Hårdkodat - tas bort vid senare tillfälle
        self.goal_x = 1.6
        self.goal_y = 0.6
        self.goal_reached = False
        self.gammla_vejen = 'NULL'

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
            self.get_logger().info(f":)")
            time.sleep(10)
            self.destroy_node()
        self.cmd_vel_pub.publish(self.tele_twist) # Publicerar till robot

    def detect_obstacle(self):
        """Obstacle Avoidance"""
        # hitta avstånt till främsta hinder
        self.obstacle_distance = min(self.scan_ranges) 

        # hitta vinkeln till närmsta hinder
        self.obstacle_direction = (self.scan_ranges).index(self.obstacle_distance)
        if self.obstacle_direction > 180:
            self.obstacle_direction -= 360 # normerar

        self.obstacle_direction = self.obstacle_direction * math.pi / 180 # omvandlar till radianer
        self.theta_gtg = math.atan2((self.goal_y - self.pose.position.y), (self.goal_x - self.pose.position.x))
        self.theta_obstacle = (self.obstacle_direction) + self.yaw #summera
        self.theta_obstacle = (self.theta_obstacle + math.pi) % (2 * math.pi) - math.pi #normera

        if self.obstacle_distance < self.stop_distance:
            # Bromsa och Navigera
            if abs(self.obstacle_direction) < (math.pi / 2): # Om roboten kollar mot hindret
                # välj riktning
                self.pic_direction()

                # Anpassa fart och riktning:
                self.tele_twist.angular.z = self.P * self.new_theta
                self.tele_twist.linear.x = self.speed * (self.obstacle_distance / self.stop_distance) * abs(math.cos(self.new_theta))
            else:
                self.get_logger().info(f"Åker mot mål")
                self.go_to_goal() # gå mot målet
        else:
            self.go_to_goal() # gå mot målet
        
    def pic_direction(self):
        # Tveksamhetstal:
        beslutsångest = math.pi / 8
        delta = ((self.theta_gtg - self.theta_obstacle) + math.pi) % (2 * math.pi) - math.pi
        if abs(delta) < beslutsångest:
            # Hinder rakt i vägen :( tar samma håll som vanligt :)
            self.get_logger().info("aktar mig :(")
            # Om han har beslutsångest igen
            if self.gammla_vejen == 'NULL':
                self.gammla_vejen = 'left'
            # Väljer gamla riktningen
            if self.gammla_vejen == 'right':
                self.new_theta = self.theta_obstacle - self.yaw - (math.pi / 2)
            else:
                self.new_theta = self.theta_obstacle - self.yaw + (math.pi / 2)
        else:
            # Bestämm snabbaste omväg
            if delta > 0:
                self.get_logger().info("Vejar vänster")
                self.new_theta = self.theta_obstacle - self.yaw + (math.pi / 2)
                self.gammla_vejen = 'left'
            else:
                self.get_logger().info("Vejar höger")
                self.new_theta = self.theta_obstacle - self.yaw - (math.pi / 2)
                self.gammla_vejen = 'right'

        self.new_theta = math.atan2(math.sin(self.new_theta), math.cos(self.new_theta)) # ;)

    def go_to_goal(self):
        """Navigating towards Goal"""
        #Avstånd
        goal_distance = math.sqrt((self.goal_x - self.pose.position.x)**2 + (self.goal_y - self.pose.position.y)**2)
        goal_threashhold = 0.1

        #Riktning
        clearance = min(self.scan_ranges)
        if clearance > 0.5:
            clearance = 0.5
        vikt = clearance / 0.5 # lidar mäter 0.5m =>
        if vikt > 0:
            # välj riktning
            self.pic_direction()

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
            self.tele_twist.linear.x = self.speed
            self.tele_twist.angular.z = self.P * e_theta # Lab2_2
            #self.tele_twist.angular.z = self.P * ((e_theta * vikt) + (self.new_theta * (1 - vikt))) # Lab2_3

    def destroy_node(self):
        """Publish zero velocity when node is destroyed"""
        self.get_logger().info("Shutting down, stopping robot...")
        stop_twist = Twist() # Default Twist has all zeros
        self.cmd_vel_pub.publish(stop_twist)
        super().destroy_node() # Call the parent class's destroy_node


def main(args=None): # Hur fungerar main?
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
