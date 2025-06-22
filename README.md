# ego-planner-v2-ros

This is a partial ros2 port of the [EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)

The main_ws has been updated to match ros2 syntax as closely as possible while retaining original logic.

```
# rm -rf build/ log/ install/
colcon build --base-paths swarm-playground/main_ws/ --symlink-install
source install/setup.bash
```

```
source install/setup.bash && unset GTK_PATH && ros2 launch ego_planner rviz.launch.py # terminal 1
source install/setup.bash && ros2 launch ego_planner single_drone_waypoints.launch.py # terminal 2
```

https://github.com/user-attachments/assets/55542737-9748-42f4-bc30-03e7faca9202