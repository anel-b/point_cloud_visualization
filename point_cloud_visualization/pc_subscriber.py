#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('point_cloud_subscriber')
        self.subscription_point_cloud = self.create_subscription(PointCloud2, 'point_cloud_topic', self.point_cloud_callback, 1)

    def point_cloud_callback(self, msg: PointCloud2):
        # Extract data from PointCloud2 message
        point_cloud_data = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 6) # Saves data in (n x 6) Matrix (n point cloud points, 6 float32 values (XYZRGB))

        # Separate XYZ and RGB values
        points_data = point_cloud_data[:, :3]
        colors_data = point_cloud_data[:, 3:]

        # Convert XYZ and RGB values to Open3D point cloud
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points_data)
        point_cloud.colors = o3d.utility.Vector3dVector(colors_data)

        # For point cloud visualization in Open3D
        o3d.visualization.draw_geometries([point_cloud])

        self.get_logger().info('Point cloud data visualized!')

def main(args=None):
    rclpy.init(args=args)
    point_cloud_subscriber = PointCloudSubscriber()
    rclpy.spin(point_cloud_subscriber)
    point_cloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
