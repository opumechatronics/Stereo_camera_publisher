cmake_minimum_required(VERSION 3.5)
project(orb_slam2davinci)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES Clang)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
#find_package(image_common REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

include_directories(include)

#Build ImagePub setting
add_executable(stereo_camera_pub
  src/stereo_camera_pub_node.cpp
  src/stereo_camera_pub_main.cpp
)
ament_target_dependencies(stereo_camera_pub 
  rclcpp 
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  camera_info_manager
  image_transport
  OpenCV)

install(TARGETS
  stereo_camera_pub
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
