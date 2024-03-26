# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0.0)

# Change project name
project (week6_teleop)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")

find_package(pipes)

# CMake will create your library, add the cpp files needed
add_library(ackerman ackerman.cpp)
target_link_libraries(ackerman pfmsconnector)

# CMake will create the cell library that you will need 
# add_library(mission mission.cpp)

# CMake will create executable add the cpp files needed
add_executable(teleop main.cpp)

# Cmake will link against the libraries in final step
target_link_libraries(teleop ackerman)
