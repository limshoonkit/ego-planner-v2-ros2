# ROS 2 package to load a PCD map

Load  a .pcd file, process it (voxelize, crop), and publish as global map.

## To Build
```
colcon build --packages-select map_processor
source install/setup.bash
```

## To Run
```
ros2 launch map_processor map_processor.launch.py
```
