cmake_minimum_required(VERSION 3.5)
project(ldlidar_stl_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Find SQLite3
find_path(SQLITE3_INCLUDE_DIR NAMES sqlite3.h)
find_library(SQLITE3_LIBRARY NAMES sqlite3)

# Find Python libraries
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Manually add matplotlibcpp header
set(MATPLOTLIBCPP_INCLUDE_DIR "/path/to/matplotlibcpp")  # Adjust this path

# Include directories
include_directories(
  include
  ${SQLITE3_INCLUDE_DIR}
  ${MATPLOTLIBCPP_INCLUDE_DIR}  # Add the directory containing matplotlibcpp.h
  ${Python3_INCLUDE_DIRS}       # Add Python include directories
)

# Add executable
add_executable(lidar_listener src/lidar_listener.cpp)

# Link libraries
target_link_libraries(lidar_listener
  ${SQLITE3_LIBRARY}
  ${Python3_LIBRARIES}  # Link against Python libraries
)

# Add dependencies
ament_target_dependencies(lidar_listener
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
)

# Install targets
install(TARGETS
  lidar_listener
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY rviz2
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
