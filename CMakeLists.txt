cmake_minimum_required(VERSION 3.5)
project(imagepub)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES Clang)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

#Build ImagePub setting
add_executable(imagepub 
  src/imagepub.cpp
  src/imagepubMain.cpp
)
ament_target_dependencies(imagepub rclcpp sensor_msgs OpenCV)

install(TARGETS
  imagepub 
  DESTINATION lib//
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
