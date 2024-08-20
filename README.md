# Point Cloud Visualization Package in ROS 2 Humble with Open3D

### Package Contents of `my_pointcloud`:

**Nodes:**
1. **`pc_publisher.py`**  
   - **Description**: Reads point cloud data from .ply file and publishes it to the "point_cloud_topic" ROS 2 topic.
2. **`pc_subscriber.py`**  
   - **Description**: Subscribes to the "point_cloud_topic" ROS 2 topic and visualizes it using Open3D library.