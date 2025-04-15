#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from builtin_interfaces.msg import Time
from carla_msgs.srv import SpawnObject
from geometry_msgs.msg import Pose, PointStamped
from carla_msgs.msg import CarlaActorInfo
import yaml
import os
import math
import uuid
import time
from sklearn.cluster import DBSCAN


class LidarToCarlaBridge(Node):
    def __init__(self):
        super().__init__('lidar_to_carla_bridge')
        self.declare_parameter('lidar_debug', False)
        self.lidar_debug = self.get_parameter('lidar_debug').get_parameter_value().bool_value

        self.declare_parameter('fov_deg', 90.0)
        self.declare_parameter('car_size_threshold', [1.4, 2.5, 4.5])
        self.declare_parameter('max_distance', 1.0)
        self.declare_parameter('metal_intensity_threshold', 12.0)
        self.declare_parameter('world_scale_ratio', 1.0)

        self.fov_deg = self.get_parameter('fov_deg').value
        self.car_size_threshold = tuple(self.get_parameter('car_size_threshold').value)
        self.max_distance = self.get_parameter('max_distance').value
        self.metal_intensity_threshold = self.get_parameter('metal_intensity_threshold').value
        self.world_scale_ratio = self.get_parameter('world_scale_ratio').value
        
        self.obstacle_pub = self.create_publisher(PointStamped, '/detected/obstacle_relative_position', 1)

        self.spawned_objects = {}

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.cloud_pub = self.create_publisher(
            PointCloud2,
            '/carla/ego_vehicle/lidar',
            10
        )

        
        self.get_logger().info(f"Lidar to Carla Bridge started. Debug mode is {self.lidar_debug}")
        if self.lidar_debug:
            self.get_logger().warn("In debug mode")
            while True:
                time.sleep(1)
                self.load_scan_from_file('/home/rfzhang/ROS-Carla/CSC791Project/lidar_to_carla/scan_sample.txt')


    def load_scan_from_file(self, filepath):
        def clean_scan_list(raw_list, fallback=12.0):
            return [
                float(x) if isinstance(x, (int, float)) and math.isfinite(x)
                else fallback
                for x in raw_list
                if isinstance(x, (int, float)) or (isinstance(x, str) and x.replace('.', '', 1).isdigit())
            ]
        with open(filepath, 'r') as f:
            docs = list(yaml.safe_load_all(f))

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
            self.scan_callback(scan_msg)

    def scan_callback(self, scan_msg: LaserScan):
        points = []
        fov_rad = math.radians(self.fov_deg)
        fov_half = fov_rad / 2

        # Compute start and end indices of the FOV
        total_angles = len(scan_msg.ranges)
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        angle_increment = (scan_msg.angle_max - angle_min) / total_angles

        # FOV range: [-fov_half, +fov_half]
        start_idx = max(0, int(( -fov_half - angle_min ) / angle_increment))
        end_idx   = min(total_angles, int(( fov_half - angle_min ) / angle_increment))

        if self.lidar_debug:
            self.get_logger().warn(f"In total {len(scan_msg.ranges)} ranges. Angle min is {angle_min}, fov angle is {fov_half}; start from index {start_idx}, end with {end_idx}")
        # In the FOV
        for i in range(start_idx, end_idx):
            r = scan_msg.ranges[i]
            if scan_msg.range_min < r < self.max_distance:
                if i < len(scan_msg.intensities) and scan_msg.intensities[i] < self.metal_intensity_threshold:
                    continue
                angle = angle_min + i * angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0
                points.append([x, y, z])

        cloud_msg = pc2.create_cloud(
            scan_msg.header,
            [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ],
            points
        )
        self.cloud_pub.publish(cloud_msg)
        self.detect_and_update_cars(np.array(points))

    # Cluster the points and detect if there's a metal object (car)
    def detect_and_update_cars(self, points):
        if len(points) == 0:
            return

        # Cluster points using DBSCAN
        clustering = DBSCAN(eps=0.15, min_samples=8).fit(points[:, :2])
        labels = clustering.labels_
        num_clusters = len(set(labels)) - (1 if -1 in labels else 0)
        # if self.lidar_debug:
        self.get_logger().warn(f"In total {len(points)} points, {num_clusters} clusters")

        for label in set(labels):
            if label == -1:
                continue
            cluster_points = points[labels == label]
            # if cluster_points.shape[0] < 5:
            #     continue

            x_vals, y_vals = cluster_points[:, 0], cluster_points[:, 1]
            width = np.max(np.abs(y_vals - np.mean(y_vals)))
            length = np.max(np.abs(x_vals - np.mean(x_vals)))
            diag = np.linalg.norm([width, length])
            self.get_logger().warn(f"Y max is {np.max(y_vals)}, Y min is {np.min(y_vals)}")
            # if width < self.car_size_threshold[0] or length < self.car_size_threshold[1] or diag > self.car_size_threshold[2]:
            #     continue

            center = np.mean(cluster_points, axis=0)
            dx = np.max(x_vals) - np.min(x_vals)
            dy = np.max(y_vals) - np.min(y_vals)
            yaw = math.atan2(dy, dx)

            # Scale to carla world size
            center_scaled = center * self.world_scale_ratio

            car_id = str(uuid.uuid4())[:8]
            
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "ego_vehicle"
            # Use the middle point
            msg.point.x = 0.5 * (np.max(x_vals) + np.min(x_vals)) * self.world_scale_ratio
            msg.point.y = 0.5 * (np.max(y_vals) + np.min(y_vals)) * self.world_scale_ratio * 0.3
            self.obstacle_pub.publish(msg)
            # Only publish one message
            break

    # Spawn a vehicle (carla actor)
    def spawn_car(self, car_id, x, y, z, yaw=0.0):
        request = SpawnObject.Request()
        request.type = "vehicle.tesla.model3"
        request.id = car_id

        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = math.cos(yaw / 2.0)
        pose.orientation.z = math.sin(yaw / 2.0)
        if self.lidar_debug:
            self.get_logger().warn(f"Trying to Spawn car at ({float(x)}, {float(y)}), car id is {car_id}")
        request.transform = pose

        request.random_pose = False
        request.attach_to = 0

        future = self.spawn_client.call_async(request)

        def callback(fut):
            try:
                result = fut.result()
                if result.id >= 0:
                    self.spawned_objects[car_id] = (x, y, z)
                    self.get_logger().info(f"Spawned car {car_id} at ({x:.2f}, {y:.2f}) with yaw {yaw:.2f}")
                else:
                    self.get_logger().error(f"Failed to spawn car: {result.error_string}")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")

        future.add_done_callback(callback)


def main(args=None):
    rclpy.init(args=args)
    node = LidarToCarlaBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
