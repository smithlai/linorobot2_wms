# Readme

## Installation
1. install ROS2
2. Install pip3
```sh
sudo apt install python3-pip
pip install numpy
pip install pandas
pip install matplotlib
pip install scikit-image
pip install scipy
```

## Test steps:

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
> Manual  
```sh
ros2 launch wms_navigation wms_navigation.launch.py rviz:=True sim:=True slam:=True
```
_OR_  
> Autonomous  
```sh
# 1
ros2 launch wms_navigation wms_navigation.launch.py rviz:=True sim:=True slam:=True map_analyzer:=True exploration_node:=False

ros2 launch myautomapping automapping2.launch.py

# or
# 2
ros2 launch wms_navigation wms_navigation.launch.py rviz:=True sim:=True slam:=True map_analyzer:=True exploration_node:=True
```

4. save the map
```sh
ros2 run nav2_map_server map_saver_cli -f wms2 --ros-args -p save_map_timeout:=10000.0
```
