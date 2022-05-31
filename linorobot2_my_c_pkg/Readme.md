# Template Example

## ament_cmake_python
`ros2 pkg create --build-type ament_cmake linorobot2_my_c_pkg`

1. CMakelists.txt
```c
    find_package(ament_cmake REQUIRED)
    find_package(ament_cmake_python REQUIRED)
    ....
    # Install Python modules
    ament_python_install_package(${PROJECT_NAME})
```
2. package.xml
```xml
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>
```

3. linorobot2_my_c_pkg/__init.py__

## To launch Navigation
`ros2 launch linorobot2_gazebo gazebo.launch.py`
`ros2 launch linorobot2_my_c_pkg my_nav2_example.launch.py rviz:=True use_sim_time:=True`
