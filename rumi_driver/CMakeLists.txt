cmake_minimum_required(VERSION 3.8)
project(rumi_driver)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libgpiod REQUIRED IMPORTED_TARGET libgpiod)

find_package(ament_cmake REQUIRED)
set(libs
        rclcpp
        geometry_msgs
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
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} PkgConfig::libgpiod ${targs})

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})
