#######################################################################
# Now we enable gtests - this section does not need to change
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)

message(STATUS "The include for simulator is ${SIMULATOR_INCLUDE}")


include_directories(
 ${GTEST_INCLUDE_DIRS}
 ${SIMULATOR_INCLUDE} #This enabled us to include simulator
 ${CMAKE_SOURCE_DIR}  #This enabled us to include files that are part of project (such as tf.h)
)

enable_testing()
###################################

# Add the unit test executable against the source code of the test
add_executable(utest utest.cpp)
# Linking to gtest and to your libraries
target_link_libraries(utest GTest::GTest GTest::Main analysis tf2 simulator ${OpenCV_LIBS})
# We can finally add the test to a list of tests to be run with make test
add_test(Q4A1 utest)
