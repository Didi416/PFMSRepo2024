# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0)

# Any ppoject name will suffice, this has connotaions when using advanced CMake Features
project (quiz1_a2)

set(BUILD_TESTS ON) #Enable all unit tests to be compiled by default

# CMake will now create the Makefiles that are needed to make an executable "hello" that requires "hello.cpp" 
add_executable(quiz1_a2 main.cpp shape.cpp rectangle.cpp)

##################################################################################################
# The below are our automated tets that will be used for marking (3 unit tests supplied)
if(BUILD_TESTS)
    add_subdirectory(test)
endif()
