cmake_minimum_required(VERSION 3.19)

project(homo)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} virtual-billboard.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})