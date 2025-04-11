# ROS2-based DBSCAN Clustering Package
![GitHub last commit](https://img.shields.io/github/last-commit/aktamaa/DBSCAN-ROS2)

ROS2-based DBSCAN Clustering made for point cloud clustering in ROS environment. This code is developed based on [DBSCAN C++ Library](https://github.com/Eleobert/dbscan.git) by [Eleobert](https://github.com/Eleobert).

## Installation
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/aktamaa/DBSCAN-ROS2.git
cd ..
colcon build
```

## Run The Node
```bash
ros2 run dbscan_ros dbscan_node --ros-args -p eps:=0.2 -p min_pts:=5 -p input_topic:=/ouster/points -p output_topic:=/clustered_points
```
