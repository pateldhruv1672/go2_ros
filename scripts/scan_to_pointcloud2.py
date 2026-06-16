#!/usr/bin/env python3
import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header

class ScanToPointCloud2(Node):
    def __init__(self):
        super().__init__("scan_to_pointcloud2")
        self.pub = self.create_publisher(PointCloud2, "/point_cloud2", 10)
        self.sub = self.create_subscription(LaserScan, "/scan", self.cb, 10)
        self.get_logger().info("Converting /scan -> /point_cloud2")

    def cb(self, scan: LaserScan):
        points = []
        angle = scan.angle_min

        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0
                points.append((x, y, z))
            angle += scan.angle_increment

        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = scan.header.frame_id or "base_scan"
        msg.height = 1
        msg.width = len(points)
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = b"".join(struct.pack("<fff", *p) for p in points)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ScanToPointCloud2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
