cmake_minimum_required(VERSION 3.14)
project(kalman-mower)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

add_executable(kalman-mower
    src/main.cpp
    src/data_generator.cpp
    src/ekf.cpp
)

target_include_directories(kalman-mower PRIVATE src)
target_link_libraries(kalman-mower Eigen3::Eigen)