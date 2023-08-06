cmake_minimum_required(VERSION 3.8)
project(rumicar)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY bin/ DESTINATION bin USE_SOURCE_PERMISSIONS)
