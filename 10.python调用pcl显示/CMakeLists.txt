cmake_minimum_required(VERSION 3.17)
project(main)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.9 REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(pybind11)
pybind11_add_module(example main.cpp)


target_link_libraries(example PRIVATE pybind11::module ${PCL_LIBRARIES})