cmake_minimum_required(VERSION 2.8.3) #Min version might not be correct
project(p3dx_robot)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)
