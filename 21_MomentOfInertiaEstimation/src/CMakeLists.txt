cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(moment_of_inertia)


if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)


find_package(PCL 1.9 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (moment_of_inertia moment_of_inertia.cpp)
target_link_libraries (moment_of_inertia ${PCL_LIBRARIES})
