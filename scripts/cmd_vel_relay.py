#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_cmd_vel_out_relay")
        self.pub = self.create_publisher(Twist, "/cmd_vel_out", 10)
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.cb, 10)
        self.get_logger().info("Relaying /cmd_vel -> /cmd_vel_out")

    def cb(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CmdVelRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
