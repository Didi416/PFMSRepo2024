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
find_package(visualization_msgs REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(pipes 3.0.5 REQUIRED)

#################################################################
# The library we are developing will be called laserprocessing
add_library(laserprocessing src/laserprocessing.cpp)
ament_target_dependencies(laserprocessing rclcpp std_msgs sensor_msgs nav_msgs std_srvs visualization_msgs)

add_library(controller src/controller.cpp)
ament_target_dependencies(controller rclcpp std_msgs sensor_msgs nav_msgs std_srvs visualization_msgs)

## This is the executable
# add_executable(sample src/main.cpp src/sample.cpp)
# ament_target_dependencies(sample rclcpp std_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs)

add_executable(ackerman src/main_ackerman.cpp src/controller.cpp src/ackerman.cpp)
ament_target_dependencies(ackerman rclcpp std_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs visualization_msgs)
add_executable(mission src/main_mission.cpp src/mission.cpp)
ament_target_dependencies(mission rclcpp std_msgs sensor_msgs nav_msgs tf2 visualization_msgs std_srvs)


target_link_libraries(ackerman controller laserprocessing audi)

install(TARGETS
  ackerman
  mission
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
