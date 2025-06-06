cmake_minimum_required(VERSION 3.5)
project(a3_skeleton)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)
else()
  add_compile_options(-Wunused-parameter)
endif()

## Find packages, remove those that are not needed
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(ament_cmake_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)

#################################################################
# The library we are developing will be called laserprocessing
add_library(laserprocessing src/laserprocessing.cpp)
ament_target_dependencies(laserprocessing rclcpp std_msgs sensor_msgs nav_msgs)

## This is the executable
add_executable(sample src/main.cpp src/sample.cpp)

## TASK 5 - Add the package to the end of the dependencies
ament_target_dependencies(sample rclcpp std_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs)

target_link_libraries(sample laserprocessing)

install(TARGETS
  sample
  laserprocessing
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    data
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)

  ament_add_ros_isolated_gtest(test_laserprocessing test/utest.cpp)
  target_link_libraries(test_laserprocessing laserprocessing)
  ament_target_dependencies(test_laserprocessing rclcpp sensor_msgs nav_msgs rosbag2_cpp)

endif()

ament_package()
