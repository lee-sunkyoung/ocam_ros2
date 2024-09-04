cmake_minimum_required(VERSION 3.5)
project(ocam)

# C++11 이상 버전 설정
add_compile_options(-std=c++11)

# ROS2 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# 설정 파일 빌드 (ROS2에서는 dynamic_reconfigure를 직접 사용하지 않으므로 관련 부분은 제외)
# generate_dynamic_reconfigure_options(
#   cfg/cam.cfg
# )

# 패키지 설정
ament_package()

# include 디렉토리 설정
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# 실행 파일 생성 및 링크
add_executable(ocam src/oCam.cpp src/withrobot_camera.cpp src/withrobot_utility.cpp)
ament_target_dependencies(ocam rclcpp std_msgs sensor_msgs cv_bridge image_transport camera_info_manager OpenCV Boost)
target_link_libraries(ocam ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} -lv4l2 -ludev)

install(TARGETS ocam
  DESTINATION lib/${PROJECT_NAME})
