# Readme
Test steps:

1. set environment

```sh
export LINOROBOT2_BASE=2wd
# export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:[your workspace]/src/linorobot2_wms/wms_navigation/models/
```

2. run gazebo for the office world
```sh
ros2 launch wms_navigation wms_gazebo.launch.py
```

3. launch navigation with slam
```sh
ros2 launch wms_navigation wms_navigation.launch.py rviz:=True sim:=True slam:=True
```

4. save the map
```sh
ros2 run nav2_map_server map_saver_cli -f wms2 --ros-args -p save_map_timeout:=10000.0
```
