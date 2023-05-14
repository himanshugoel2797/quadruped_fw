cmake_minimum_required(VERSION 3.1)
project(quadruped_fw)

set(CMAKE_CXX_STANDARD 14)

find_package(PiPCA9685 REQUIRED)

add_executable(fw main.cpp)
target_link_libraries(fw ${PiPCA9685_LIBRARY})