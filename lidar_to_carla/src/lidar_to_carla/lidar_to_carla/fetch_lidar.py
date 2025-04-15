#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from builtin_interfaces.msg import Time
import yaml
import os
import math


class LidarToCarlaBridge(Node):
    def __init__(self):
        super().__init__('lidar_to_carla_bridge')
        
        # Subscribe to RPLidar scans
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Publish point cloud for Carla ROS Bridge
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            '/carla/ego_vehicle/lidar',
            10)
        
        if os.getenv('LIDAR_DEBUG'):
            self.load_scan_from_file('/home/rfzhang/ROS-Carla/CSC791Project/lidar_to_carla/scan_sample.txt')

        self.get_logger().info("Lidar to Carla Bridge started.")


    def load_scan_from_file(self, filepath):
        def clean_scan_list(raw_list, fallback=12.0):
            return [
                float(x) if isinstance(x, (int, float)) and math.isfinite(x)
                else fallback
                for x in raw_list
                if isinstance(x, (int, float)) or (isinstance(x, str) and x.replace('.', '', 1).isdigit())
            ]
        with open(filepath, 'r') as f:
            docs = list(yaml.safe_load_all(f))  # Load all messages

        self.get_logger().info(f"Loaded {len(docs)} scan messages from file")

        for i, data in enumerate(docs[:-1]):
            scan_msg = LaserScan()
            scan_msg.header.frame_id = data['header']['frame_id']

            stamp = Time()
            stamp.sec = data['header']['stamp']['sec']
            stamp.nanosec = data['header']['stamp']['nanosec']
            scan_msg.header.stamp = stamp

            scan_msg.angle_min = data['angle_min']
            scan_msg.angle_max = data['angle_max']
            scan_msg.angle_increment = data['angle_increment']
            scan_msg.time_increment = data['time_increment']
            scan_msg.scan_time = data['scan_time']
            scan_msg.range_min = data['range_min']
            scan_msg.range_max = data['range_max']
            scan_msg.ranges = clean_scan_list(data['ranges'], fallback=data.get('range_max', 12.0))
            scan_msg.intensities = clean_scan_list(data.get('intensities', []), fallback=0.0)

            self.get_logger().info(f"Feeding scan {i+1}/{len(docs)} to callback")
            self.scan_callback(scan_msg)

    def scan_callback(self, scan_msg: LaserScan):
        self.get_logger().info(f"Received LaserScan with {len(scan_msg.ranges)} ranges")

        points = []
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if scan_msg.range_min < r < scan_msg.range_max:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                z = 0.0
                points.append([x, y, z])
            angle += scan_msg.angle_increment

        header = scan_msg.header
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_msg = pc2.create_cloud(header, fields, points)
        self.cloud_pub.publish(cloud_msg)

        if points:
            self.get_logger().info(f"Published {len(points)} points. First point: x={points[0][0]:.2f}, y={points[0][1]:.2f}")
        else:
            self.get_logger().warn("No valid lidar points to publish.")

def main(args=None):
    rclpy.init(args=args)
    node = LidarToCarlaBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
