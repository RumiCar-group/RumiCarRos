cmake_minimum_required(VERSION 3.16)
project(rumi_lane)

find_package(ament_cmake REQUIRED)
find_package(OpenCV)
set(libs
        cv_bridge
        image_transport
        rclcpp
        #
        geometry_msgs
        sensor_msgs
        std_msgs
)
foreach(lib IN LISTS libs)
    find_package(${lib} REQUIRED)
    list(APPEND targs ${${lib}_TARGETS})
endforeach()

ament_package()

file(GLOB_RECURSE src src/*.cpp)
add_executable(${PROJECT_NAME} ${src})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_23)
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${targs} ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
