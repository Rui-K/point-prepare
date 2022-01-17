cmake_minimum_required(VERSION 3.5 FATAL_ERROR)


project(ground_extract)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (ground_extract ground_extract.cpp)
target_link_libraries (ground_extract ${PCL_LIBRARIES})
