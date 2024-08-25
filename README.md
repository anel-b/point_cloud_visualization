# Point Cloud Visualization Package in ROS 2 Humble with Open3D

### Package Contents of `point_cloud_visualization`:

**2 Nodes:**
* **pc_publisher.py**  
   * Reads point cloud data from .ply file and publishes it to the "point_cloud_topic" ROS 2 topic.
* **pc_subscriber.py**  
   * Subscribes to the "point_cloud_topic" ROS 2 topic and visualizes it using Open3D library.