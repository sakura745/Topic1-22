cmake_minimum_required(VERSION 3.10)
project(Topic_2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

include_directories(/usr/include/eigen3)

add_executable(${PROJECT_NAME} main.cpp)