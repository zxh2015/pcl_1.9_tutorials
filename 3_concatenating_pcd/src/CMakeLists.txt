cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(concatenate_clouds)

add_definitions(-std=c++11)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

find_package(PCL 1.9 REQUIRED)

#set(PCL_DIR /home/dl/Downloads/pcl-pcl-1.9.1/build/PCLConfig.cmake)
find_package(PCL 1.9 REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS})

add_executable(concatenate_cloud concatenate_clouds.cpp)
target_link_libraries(concatenate_cloud ${PCL_LIBRARIES})
