cmake_minimum_required(VERSION 3.16)
project(rumi_nav)

find_package(ament_cmake REQUIRED)
set(libs
        rclcpp
        sensor_msgs
)
foreach(lib IN LISTS libs)
    find_package(${lib} REQUIRED)
    list(APPEND targs ${${lib}_TARGETS})
endforeach()

ament_package()

file(GLOB_RECURSE src src/*.cpp)
add_executable(${PROJECT_NAME} ${src})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} ${targs})

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
