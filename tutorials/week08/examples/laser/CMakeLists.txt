# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0.1)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (laser_data LANGUAGES CXX VERSION 3.0.1)

# This is added to c++171 features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

## Include the location of pfms headers
find_package(pipes 3.0.4 REQUIRED)

# Executables
add_executable(laser_data main.cpp laser.cpp)

# Link the library
target_link_libraries(laser_data pfmsconnector)

