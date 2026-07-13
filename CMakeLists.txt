cmake_minimum_required(VERSION 3.10)
project(IMU LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)

add_library(imu_can
    IMU/IMU_can.cpp
)

target_include_directories(imu_can
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/IMU
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/math
)

target_link_libraries(imu_can PUBLIC pthread Eigen3::Eigen)

add_executable(imu_reader
    src/main.cpp
)

target_link_libraries(imu_reader PRIVATE imu_can)
