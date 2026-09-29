cmake_minimum_required(VERSION 3.8)
project(my_robot_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
