
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(gpu_test)

find_package(PCL 1.7 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Searching CUDA
find_package(CUDA)

#Include the FindCUDA script
include(FindCUDA)


cuda_add_executable (gpu_test gpu_test.cpp)
target_link_libraries (gpu_test ${PCL_LIBRARIES})
