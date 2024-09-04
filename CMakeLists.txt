cmake_minimum_required(VERSION 3.5)
project(ocam)

# Use C++11 standard
add_definitions(-std=c++11)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
#find_package(dynamic_reconfigure REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${camera_info_manager_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Add executable
add_executable(ocam src/oCam.cpp src/withrobot_camera.cpp src/withrobot_utility.cpp)
ament_target_dependencies(ocam rclcpp std_msgs sensor_msgs cv_bridge image_transport camera_info_manager OpenCV Boost)

# Link libraries
target_link_libraries(ocam
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${camera_info_manager_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  -lv4l2
  -ludev
)

# Install executable
install(TARGETS ocam
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Ament package
ament_package()
