cmake_minimum_required(VERSION 3.17)
project(main)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.9 REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
#
find_package(Open3D REQUIRED)
# link_directories must be before add_executable
link_directories(${Open3D_LIBRARY_DIRS})

add_executable(main main.cpp)
target_link_libraries(main ${PCL_LIBRARIES} ${Open3D_LIBRARIES})