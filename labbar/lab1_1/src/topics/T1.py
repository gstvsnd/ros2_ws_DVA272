#! /usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int8

class Talker(Node):
    def __init__(self):
        super().__init__("talker1")
        self.i = 6
        self.pub = self.create_publisher(Int8, "chatter1", 10)
        timer_period = 0.5
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Int8()
        msg.data = self.i
        
        self.i = (self.i + 2) %20 
        self.get_logger().info('Publishing: {}'.format(msg.data))

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = Talker()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
