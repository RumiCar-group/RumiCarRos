cmake_minimum_required(VERSION 3.8)
project(rumicar_kuro)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
