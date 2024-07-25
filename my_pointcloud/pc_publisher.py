#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import open3d as o3d
import numpy as np
import glob
import os

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('point_cloud_publisher')
        self.publisher_point_cloud = self.create_publisher(PointCloud2, 'point_cloud_topic', 1)

        base_matrix = np.array([[-1.000,  0.000,  0.000,  0.358],
                                [ 0.000,  1.000,  0.000,  0.030],
                                [ 0.000,  0.000, -1.000,  0.006],
                                [ 0.000,  0.000,  0.000,  1.000]])

        transform_matrix = np.array([[ 0.5357,  0.5685, -0.6244,  0.5918],
                                     [-0.8444,  0.3671, -0.3902,  0.6178],
                                     [ 0.0074,  0.7363,  0.6767, -0.9096],
                                     [ 0.0000,  0.0000,  0.0000,  1.0000]])

        rotation_matrix = np.array([[ 1.000,  0.000,  0.000,  0.140],
                                    [ 0.000, -1.000,  0.000,  0.040],
                                    [ 0.000,  0.000, -1.000, -0.040],
                                    [ 0.000,  0.000,  0.000,  1.000]])

        # Transformation matrix from camera frame to robot base frame
        self.transformation = base_matrix @ transform_matrix @ rotation_matrix

        # Get all .ply files from path
        self.path = '/home/<username>/path/folder'
        self.files = sorted(glob.glob(os.path.join(self.path, '*.ply')))
        self.index = 0

        # Timer for publishing point cloud data every 0.03 seconds
        self.timer = self.create_timer(0.03, self.publish_point_cloud)

    def remove_background(self, point_cloud: o3d.geometry.PointCloud):
        # Crop point cloud to workspace
        min_bound = np.array([-0.4, -0.9, -0.03])
        max_bound = np.array([1.2, 0.9, 1.4])
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
        point_cloud = point_cloud.crop(bbox)

        return point_cloud

    def publish_point_cloud(self):
        # Get all .ply files from path and read point cloud data from current file
        file = self.files[self.index]
        self.index = (self.index + 1) % len(self.files)
        point_cloud = o3d.io.read_point_cloud(file)

        # Point cloud preprocessing
        point_cloud = point_cloud.uniform_down_sample(every_k_points=20)
        point_cloud = point_cloud.transform(self.transformation)
        point_cloud = self.remove_background(point_cloud)

        # Save XYZ and RGB values into numpy arrays
        points_data = np.asarray(point_cloud.points)
        colors_data = np.asarray(point_cloud.colors)
        point_cloud_data = np.concatenate((points_data.astype(np.float32), colors_data.astype(np.float32)), axis=1)

        # Publish point cloud data as PointCloud2 message
        msg = PointCloud2()
        msg.header = Header()
        msg.header.frame_id = 'world'
        msg.height = 1
        msg.width = len(point_cloud_data)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='g', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='b', offset=20, datatype=PointField.FLOAT32, count=1)]
        msg.is_bigendian = False
        msg.point_step = np.dtype(np.float32).itemsize * 6 # 32-bit float takes 8 bytes * 6 floats (XYZRGB values)
        msg.row_step = msg.point_step * len(point_cloud_data)
        msg.data = point_cloud_data.tobytes()
        msg.is_dense = False
        self.publisher_point_cloud.publish(msg)

        self.get_logger().info('Point cloud data published!')

def main(args=None):
    rclpy.init(args=args)
    point_cloud_publisher = PointCloudPublisher()
    rclpy.spin(point_cloud_publisher)
    point_cloud_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
