# The minimum version of CMake Required
cmake_minimum_required (VERSION 3.0)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (quiz1_a1)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(BUILD_TESTS ON) #Enable all unit tests to be compiled by default
set(MARKING_TESTS OFF) #When marking tests are deposited after deadline (you will be notified via teams) you will be able to turn this ON

# To add executables provide the executable name followed by a list of source files
# There must be exactly one function called main in each executable
add_executable(quiz1_a1 main.cpp person.cpp processing.cpp)

##################################################################################################
# The below is our an example of our automated tets for marking
# Refer canvas Week 04 - Unit Tests to install the unit testing framework if you wish
#
# We have only provided one example, whereas on submission we will test every function you develop
# To enable testsing you need to pass a switch to cmake
# cmake .. -DBUILD_TESTS=ON
# then run make as usual
## OR
# comment out with a # on lines 23 and 25 (so you can see the code in qtcreator)
#
# finally, tests can be run from build directory (for this example ./test/utest
if(BUILD_TESTS)
    add_subdirectory(test)
endif()

# Marking tests are deposited after deadline (you will be notified via teams), this directory does not estisr
if(MARKING_TESTS)
    add_subdirectory(marking)
endif()
