cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(narf_feature_extraction)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)


find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable (narf_feature_extraction narf_feature_extraction.cpp)
target_link_libraries (narf_feature_extraction ${PCL_LIBRARIES})
