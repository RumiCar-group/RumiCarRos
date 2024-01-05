cmake_minimum_required(VERSION 3.16)
project(rumi_driver)

find_package(ament_cmake REQUIRED)
find_package(rumi_hw REQUIRED)
set(libs
# build depend
        rclcpp
        tf2
        tf2_geometry_msgs
# depend
        geometry_msgs
        nav_msgs
        sensor_msgs
        std_msgs
        tf2_ros
)
foreach(lib IN LISTS libs)
    find_package(${lib} REQUIRED)
    list(APPEND targs ${${lib}_TARGETS})
endforeach()

ament_package()

file(GLOB_RECURSE src src/*.cpp)
add_executable(${PROJECT_NAME} ${src})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} ${targs} rumi::hw)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
