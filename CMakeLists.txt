cmake_minimum_required(VERSION 3.8)
project(multiple_object_tracking_lidar)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(ament_cmake REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this

add_executable(multi_object_tracking_lidar src/main.cpp)

target_include_directories(multi_object_tracking_lidar PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(multi_object_tracking_lidar PUBLIC /opt/ros/humble/include/visualization_msgs)


target_link_libraries(multi_object_tracking_lidar
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(multi_object_tracking_lidar
  pcl_ros
  rclcpp
  sensor_msgs
  OpenCV
)

install(TARGETS multi_object_tracking_lidar
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()