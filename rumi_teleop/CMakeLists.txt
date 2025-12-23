cmake_minimum_required(VERSION 3.8)

project(rumi_teleop)

find_package(ament_cmake REQUIRED)
set(libs
        rclcpp
        geometry_msgs
        sensor_msgs
        std_msgs
)
foreach(lib IN LISTS libs)
    find_package(${lib} REQUIRED)
    list(APPEND targs ${${lib}_TARGETS})
endforeach()

ament_package()

add_executable(${PROJECT_NAME} src/rumi_teleop_node.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_23)
target_link_libraries(${PROJECT_NAME} ${targs})

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
