# Readme
Test steps:

1. set environment

```
export LINOROBOT2_BASE=2wd
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/lino_usi_ws/src/linorobot2_wms/wms_navigation/models/
```

2. run gazebo for our office
```
ros2 launch wms_navigation wms_gazebo.launch.py
```

3. launch navigation with slam
```
ros2 launch wms_navigation wms_navigation.launch.py rviz:=True sim:=True slam:=True
```

4. save the map
```
ros2 run nav2_map_server map_saver_cli -f wms2 --ros-args -p save_map_timeout:=10000.0
```
