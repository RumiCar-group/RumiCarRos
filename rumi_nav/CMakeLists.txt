cmake_minimum_required(VERSION 3.16)
project(rumi_nav)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
