cmake_minimum_required(VERSION 3.10)

project(Topic_1)
project(text)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)


find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})


add_executable(Topic_1 main.cpp)
target_link_libraries(Topic_1 ${OpenCV_LIBS})

add_executable(text main_o.cpp)