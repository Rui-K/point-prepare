cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
add_definitions("-std=c++11")

project(downsample)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (downsample downsample.cpp)
target_link_libraries (downsample ${PCL_LIBRARIES})
