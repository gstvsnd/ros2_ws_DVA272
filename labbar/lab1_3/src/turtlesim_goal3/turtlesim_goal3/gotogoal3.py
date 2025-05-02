#!/usr/bin/env python3
import threading
import time
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from math import sqrt, atan2, pi
from geometry_msgs.msg import Twist
import ast


class TurtleBot(Node):
    def __init__(self):
        super().__init__("turtlesim_goal3")

        self.declare_parameter('waypoints', [3.7, 4.0, 3.6, 4.4, 3.4, 4.6, 3.0, 4.7, 2.6, 4.6, 2.4, 4.4, 2.3, 4.0, 2.4, 3.6, 2.6, 3.4, 3.0, 3.3, 3.4, 3.4, 3.6, 3.6, 3.7, 3.4, 4.7, 3.0, 4.6, 3.4, 4.4, 3.6, 4.0, 3.7, 3.6, 3.6, 3.4, 3.4, 3.3, 3.0, 3.4, 2.6, 3.6, 2.4, 4.0, 2.3, 4.4, 2.4, 4.6, 2.6, 4.7, 3.0, 4.7, 3.5, 6.0, 5.0, 6.5, 5.5, 6.4, 5.8, 6.2, 5.9, 6.0, 6.0, 5.8, 5.9, 5.6, 5.8, 5.5, 5.5, 5.6, 5.2, 5.8, 5.1, 6.0, 5.0, 6.2, 5.1, 6.4, 5.2]
)
        #"3.7, 4.0, 3.6, 4.4; 3.4, 4.6; 3.0, 4.7; 2.6, 4.6; 2.4, 4.4; 2.3, 4.0; 2.4, 3.6; 2.6, 3.4; 3.0, 3.3; 3.4, 3.4; 3.6, 3.6; 3.7, 3.4; 4.7, 3.0; 4.6, 3.4; 4.4, 3.6; 4.0, 3.7; 3.6, 3.6; 3.4, 3.4; 3.3, 3.0; 3.4, 2.6; 3.6, 2.4; 4.0, 2.3; 4.4, 2.4; 4.6, 2.6; 4.7, 3.0; 4.7, 3.5; 6.0, 5.0; 6.5, 5.5; 6.4, 5.8; 6.2, 5.9; 6.0, 6.0; 5.8, 5.9; 5.6, 5.8; 5.5, 5.5; 5.6, 5.2; 5.8, 5.1; 6.0, 5.0; 6.2, 5.1; 6.4, 5.2"
        # Get waypoints as a string parameter
        self.waypoints = self.get_parameter('waypoints').get_parameter_value().double_array_value
        #self.waypoints = [list(map(float, pair.split(','))) for pair in waypoints_string.split(';')]
        # Initialize the list for storing waypoints
        #self.waypoints = []
        #temp_string = ""


        #for i in range(len(waypoints_string)):
        #    if waypoints_string[i] == ',' or waypoints_string[i] == ';':
        #        if waypoints_string[i-4] == ' ':
        #            temp_stringlist = [waypoints_string[i-3], waypoints_string[i-2], waypoints_string[i-1]]
        #        else:    
        #            temp_stringlist = [waypoints_string[i-4], waypoints_string[i-3], waypoints_string[i-2], waypoints_string[i-1]] # Här ska vi fixa så de kanske kan bli fyra grejs
        #        temp_string = ''.join(temp_stringlist)#Här gör vi en lista med trängar till en sträng
        #        self.waypoints.append(float(temp_string))
        #self.waypoints.append(3.0)
        #self.waypoints.append(3.0)

        #self.get_logger().info(f"Parsed Waypoints: {self.waypoints}")


        # Publisher for velocity commands
        self.velocity_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        
        # Subscriber for turtle's position
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.update_pose, 10
        )

        # Movement parameters
        self.max_linear_speed = 1.5
        self.min_linear_speed = 0.3
        self.angular_speed_factor = 4.0

        # Setup control timer (10Hz)
        self.timer = self.create_timer(0.1, self.controller_callback)

        # Setup turtle state
        self.pose = Pose()
        self.goal_pose = Pose()
        self.moving_to_goal = False
        self.distance_tolerance = 0.1

        # Index to track the current waypoint
        self.waypoint_index = 0

        # For position logging
        self.last_log_time = 0.0

        # Start moving to the first waypoint
        self.move_to_next_waypoint()

    def update_pose(self, data):
        """Store the turtle's current position"""
        self.pose = data

    def euclidean_distance(self):
        """Distance between current position and goal"""
        # FIXME: Mathematical Error in Distance Calculation
        return sqrt(
            (self.goal_pose.x - self.pose.x) ** 2
            + (self.goal_pose.y - self.pose.y) ** 2
        )

    def calculate_linear_velocity(self):
        """Calculate forward speed with deceleration near goal"""
        distance = self.euclidean_distance()

        # Deceleration zone is twice the tolerance
        decel_zone = self.distance_tolerance * 2.0

        if distance < decel_zone:
            # Decelerate as we approach the goal
            speed = self.min_linear_speed + (
                self.max_linear_speed - self.min_linear_speed
            ) * (distance / decel_zone)
        else:
            # Normal speed
            speed = self.max_linear_speed

        # Ensure speed stays within bounds
        return max(self.min_linear_speed, min(speed, self.max_linear_speed))

    def calculate_steering_angle(self):
        """Angle toward goal"""
        return atan2(self.goal_pose.y - self.pose.y, self.goal_pose.x - self.pose.x)

    def calculate_angular_velocity(self):
        """Calculate rotational speed for steering"""
        # Angle difference between current heading and goal
        angle_diff = self.calculate_steering_angle() - self.pose.theta

        # Normalize angle between -pi and pi
        while angle_diff > pi:
            angle_diff -= 2 * pi
        while angle_diff < -pi:
            angle_diff += 2 * pi

        # Apply proportional control with stronger corrections for larger angles
        if abs(angle_diff) < 0.1:  # Small corrections
            return angle_diff * self.angular_speed_factor * 0.8
        elif abs(angle_diff) < 0.5:  # Medium corrections
            return angle_diff * self.angular_speed_factor
        else:  # Large corrections
            return angle_diff * self.angular_speed_factor * 1.2

    def controller_callback(self):
        """Main control loop - called 10 times per second"""
        if not self.moving_to_goal:
            return

        # Log current position periodically (once per second)
        current_time = time.time()
        if current_time - self.last_log_time >= 1.0:
            self.get_logger().info(
                f"Current position: x={self.pose.x:.2f}, y={self.pose.y:.2f}, distance_to_goal={self.euclidean_distance():.2f}"
            )
            self.last_log_time = current_time

        # If we're close enough to the goal, stop moving
        if self.euclidean_distance() < self.distance_tolerance:
            # Stop the turtle
            vel_msg = Twist()
            self.velocity_publisher.publish(vel_msg)

            # Mark goal as reached
            self.moving_to_goal = False
            self.get_logger().info("Goal reached!")
            time.sleep(1)
            self.get_logger().info(
                f"Current position: x={self.pose.x:.2f}, y={self.pose.y:.2f}, distance_to_goal={self.euclidean_distance():.2f}"
            )
            # Start moving to the first waypoint
            self.move_to_next_waypoint()

        # We need to keep moving toward the goal
        vel_msg = Twist()

        # Calculate forward speed
        linear_velocity = self.calculate_linear_velocity()

        # Adjust speed during turns
        angular_diff = abs(self.calculate_steering_angle() - self.pose.theta)
        while angular_diff > pi:
            angular_diff = abs(angular_diff - 2 * pi)

        # Slow down when turning sharply
        turn_factor = 1.0
        if angular_diff > 0.5:  # ~30 degrees
            turn_factor = max(0.4, 1.0 - angular_diff / pi)

        # Set the velocity commands
        vel_msg.linear.x = linear_velocity * turn_factor
        vel_msg.angular.z = self.calculate_angular_velocity()

        # Send command to the turtle
        self.velocity_publisher.publish(vel_msg)

    def move_to_next_waypoint(self):
        """Move to the next waypoint in the list."""
        if self.waypoint_index < (len(self.waypoints)):
            # Set the current goal
            self.goal_pose.x = self.waypoints[(self.waypoint_index)]
            self.goal_pose.y = self.waypoints[(self.waypoint_index)+1]# HÄR FIXA!!!
            self.goal_pose.x = self.goal_pose.x %11
            self.goal_pose.y = self.goal_pose.y %11
            if self.goal_pose.x < 0.5:
                self.goal_pose.x = 1.0
            if self.goal_pose.y < 0.5:
                self.goal_pose.y = 1.0
            # Increment the waypoint index for the next goal
            print("Hurra nu har vi nått en koordinat!")
            self.waypoint_index += 2

            # Start moving towards the goal
            self.moving_to_goal = True
            self.get_logger().info(f"Moving to waypoint: {self.goal_pose.x}, {self.goal_pose.y}")
        else:
            self.get_logger().info("All waypoints reached!")


def main():
    # Initialize ROS
    rclpy.init()
    turtlebot = TurtleBot()

    # Run ROS in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(turtlebot,))
    spin_thread.daemon = True
    spin_thread.start()

    try:
        while True:
            choice = input("\n1. Go to position, 2. Exit: ")

            if choice == "1":
                try:
                    x = float(input("X: ")) %11.1
                    y = float(input("Y: ")) %11.1

                    # Set the goal and start moving
                    turtlebot.get_logger().info(f"Moving to: x={x}, y={y}")
                    turtlebot.goal_pose.x = x
                    turtlebot.goal_pose.y = y
                    turtlebot.moving_to_goal = True

                    # Wait for movement to complete
                    while turtlebot.moving_to_goal:
                        time.sleep(0.5)

                except ValueError:
                    print("Please enter valid numbers")

            elif choice == "2":
                break

            else:
                print("Invalid choice")

    except KeyboardInterrupt:
        pass

    finally:
        turtlebot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
