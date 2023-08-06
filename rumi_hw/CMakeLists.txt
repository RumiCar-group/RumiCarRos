cmake_minimum_required(VERSION 3.16)
project(rumi_hw)

MESSAGE(STATUS "Using toolchain file: ${CMAKE_TOOLCHAIN_FILE}")

find_package(PkgConfig REQUIRED)
pkg_check_modules(libgpiod REQUIRED IMPORTED_TARGET libgpiod)

file(GLOB_RECURSE src src/*.cpp include/*.hpp)
add_library(${PROJECT_NAME} ${src})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PRIVATE PkgConfig::libgpiod)

add_library(rumi::hw ALIAS ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES EXPORT_NAME hw)

include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
        INSTALL_DESTINATION share/${PROJECT_NAME}
        NO_SET_AND_CHECK_MACRO
        NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME})
install(EXPORT ${PROJECT_NAME}
        NAMESPACE rumi::
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h*"
)
install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
        DESTINATION share/${PROJECT_NAME}
)
