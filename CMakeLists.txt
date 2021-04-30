cmake_minimum_required(VERSION 3.5)
project(ros2_latency)

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
find_package(PCL REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add executables
add_executable(repeater src/repeater.cpp)
add_executable(measure src/measure.cpp)
add_executable(source src/source.cpp)

include_directories(
    include()
    ${PCL_INCLUDE_DIRS}
)

# Add executable dependencies
ament_target_dependencies(repeater rclcpp sensor_msgs pcl_conversions tf2_geometry_msgs)
ament_target_dependencies(measure rclcpp sensor_msgs pcl_conversions tf2_geometry_msgs)
ament_target_dependencies(source rclcpp sensor_msgs pcl_conversions tf2_geometry_msgs)
target_link_libraries(repeater ${Boost_LIBRARIES})
target_link_libraries(measure ${Boost_LIBRARIES})
target_link_libraries(source ${Boost_LIBRARIES})

install(TARGETS
  # Add installed executables
  repeater
  measure
  source
  DESTINATION lib/${PROJECT_NAME})
  
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ros2_latency/repeater.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
