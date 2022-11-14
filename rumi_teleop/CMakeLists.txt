cmake_minimum_required(VERSION 3.8)
project(rumi_teleop)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
