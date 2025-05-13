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
        self.speed = 0.5
        self.P = 1

        # Goal - Hårdkodat - tas bort vid senare tillfälle
        self.goal_x = 0.5
        self.goal_y = -1.5
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
        # hitta avstånt till närmaste hinder
        self.obstacle_distance = min(self.scan_ranges)
        self.obstacle_direction = self.scan_ranges.index(self.obstacle_distance)

        if self.obstacle_direction > 180:
            self.obstacle_direction -= 360  #normering (-180 - 180)

        self.obstacle_direction = math.radians(self.obstacle_direction) #radianer till obs från robot

        self.theta_gtg = math.atan2(self.goal_y - self.pose.position.y, self.goal_x - self.pose.position.x) #radianer till goal i världs kordinat

        self.theta_obstacle = self.yaw + self.obstacle_direction
        self.theta_obstacle = math.atan2(math.sin(self.theta_obstacle), math.cos(self.theta_obstacle)) #radianer till obs i världskortinat

        if self.obstacle_distance < self.stop_distance:
            if abs(self.obstacle_direction) < math.pi / 3:  #om robot har hinder 180 framför sig
                self.pic_direction()
                scaling = 0.0#self.obstacle_distance / (2*self.stop_distance)  #2.2 sätt till 0
                self.tele_twist.linear.x = self.speed * scaling * abs(math.cos(self.new_theta))
                self.tele_twist.angular.z = self.new_theta
            else:
                self.go_to_goal()
        elif self.obstacle_distance < 0.2:
            if abs(self.obstacle_direction) < math.pi / 2:  #om robot har hinder 180 framför sig
                self.tele_twist.linear.x = 0.0#self.speed * scaling * abs(math.cos(self.new_theta))
                self.pic_direction()
                #scaling = self.obstacle_distance / (2*self.stop_distance)
                self.tele_twist.angular.z = self.new_theta
            else:
                self.go_to_goal()
        else:
            self.go_to_goal()
        
    def pic_direction(self):
        # Tveksamhetstal:
        decision_threshold = math.pi / 8  #undviker velande när mål och obs i samma riktning ish
        delta = (self.theta_gtg - self.theta_obstacle + math.pi) % (2 * math.pi) - math.pi

        if abs(delta) < decision_threshold:
            self.get_logger().info("Velig: Tar gamla svängen")
            if self.gammla_vejen == 'NULL':
                self.gammla_vejen = 'left'
            if self.gammla_vejen == 'right':
                self.new_theta = self.theta_obstacle - self.yaw - 2*(math.pi / 4)
            else:
                self.new_theta = self.theta_obstacle - self.yaw + 2*(math.pi / 4)
        else:
            if delta > 0:
                self.get_logger().info("Svänger vänster")
                self.new_theta = self.theta_obstacle - self.yaw + 2*(math.pi / 4)
                self.gammla_vejen = 'left'
            else:
                self.get_logger().info("Svänger höger")
                self.new_theta = self.theta_obstacle - self.yaw - 2*(math.pi / 4)
                self.gammla_vejen = 'right'

        self.new_theta = math.atan2(math.sin(self.new_theta), math.cos(self.new_theta)) #normering (-pi - pi)

    def go_to_goal(self):
        """Navigating towards Goal"""
        #Avstånd
        dx = self.goal_x - self.pose.position.x  # X ifrån mål
        dy = self.goal_y - self.pose.position.y  # Y ifrån mål
        goal_distance = math.hypot(dx, dy)  # Bellop på vektor till mål
        goal_threshold = 0.1
        max_visual = 0.5

        #Riktning
        clearance = max(0.0, min(self.scan_ranges) - self.stop_distance)
        clearance = min(clearance, max_visual - self.stop_distance)
        vikt = clearance / (max_visual - self.stop_distance)  # Tal mellan 0-1

        self.pic_direction()
        
        theta_gtg = math.atan2(dy, dx)
        e_theta = theta_gtg - self.yaw
        e_theta = math.atan2(math.sin(e_theta), math.cos(e_theta))
        blended_theta = (e_theta * vikt) + ((self.new_theta) * (1 - vikt))
        if vikt < 0.3:
            vikt = 0.3
        
        #Navigering
        if goal_distance < goal_threshold:
            self.goal_reached = True
            self.tele_twist.linear.x = 0.0
            self.tele_twist.angular.z = 0.0
        else:
            if goal_distance < 0.35:
                self.tele_twist.linear.x = self.speed * 0.5
            else:
                self.tele_twist.linear.x = self.speed * vikt
            self.tele_twist.angular.z = self.P * e_theta # Lab2.2
            #self.tele_twist.angular.z = self.P * 2 * blended_theta # Lab2.3

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