cmake_minimum_required(VERSION 3.5)
project(lidar_pub)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ydlidar_sdk REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${YDLIDAR_SDK_INCLUDE_DIRS})


add_executable(lidar_pub src/lidar_pub.cpp)
ament_target_dependencies(lidar_pub rclcpp sensor_msgs)
target_link_libraries(lidar_pub ${YDLIDAR_SDK_LIBRARIES})


install(TARGETS
  lidar_pub
  DESTINATION lib/${PROJECT_NAME})

ament_package()