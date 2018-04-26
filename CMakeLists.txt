cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(Segmentation_DoN)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.5 REQUIRED)

# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(project_SOURCES main)
add_executable(${PROJECT_NAME} ${project_SOURCES})
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})