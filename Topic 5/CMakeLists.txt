cmake_minimum_required( VERSION 3.10 )
project( rotationTransforms )


set(CMAKE_CXX_STANDARD   14)
set(CMAKE_BUILD_TYPE Release)

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

add_executable( rotationTransforms rotationTransforms.cpp )
