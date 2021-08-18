cmake_minimum_required(VERSION 3.10)

project(Trajectory)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(Pangolin)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(/usr/include/eigen3)

add_executable(${PROJECT_NAME} draw_trajectory.cpp)
target_link_libraries(${PROJECT_NAME}
        ${Pangolin_LIBRARIES})
#        ${Sophus_LIBS})
