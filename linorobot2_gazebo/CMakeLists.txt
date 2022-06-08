cmake_minimum_required(VERSION 3.5)
project(linorobot2_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# File "/opt/ros/galactic/share/gazebo_ros/launch/gzserver.launch.py", line 28, in <module>
# from scripts import GazeboRosPaths
# ImportError: cannot import name 'GazeboRosPaths' from 'scripts' (/home/smith/dev_ws/install/linorobot2_gazebo/lib/python3.8/site-packages/scripts/__init__.py)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/command_timeout.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
