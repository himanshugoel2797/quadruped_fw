cmake_minimum_required(VERSION 3.1)
project(quadruped_fw)

set(CMAKE_CXX_STANDARD 14)

find_package(PiPCA9685 REQUIRED)

add_executable(fw src/main.cpp)
target_link_libraries(fw ${PiPCA9685_LIBRARY} pigpio pthread)
target_include_directories(fw PUBLIC ${PiPCA9685_INCLUDE_DIR} glm)