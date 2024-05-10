cmake_minimum_required(VERSION 3.5)
project(a3_support)

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

## Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(goals_logger src/goals_logger.cpp )
ament_target_dependencies(goals_logger rclcpp geometry_msgs visualization_msgs)

add_executable(goals_publisher src/goals_publisher.cpp )
ament_target_dependencies(goals_publisher rclcpp geometry_msgs visualization_msgs)

install(TARGETS
  goals_logger
  goals_publisher
  DESTINATION lib/${PROJECT_NAME})

# We are also going to install the data directory that has sample goals.
install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()