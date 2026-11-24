cmake_minimum_required(VERSION 3.8)
project(alphabot2_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf worlds config launch rviz
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/head_to_pantilt.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
