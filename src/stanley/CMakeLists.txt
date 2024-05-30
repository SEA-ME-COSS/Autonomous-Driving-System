cmake_minimum_required(VERSION 3.5)
project(stanley)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# ROS 2 패키지 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

find_package(Boost REQUIRED)

ament_package()

include_directories(
  include/${PROJECT_NAME}
  include/${PROJECT_NAME}/dbcppp/src
  include/${PROJECT_NAME}/dbcppp/include
)

FILE(GLOB CANProtocol include/${PROJECT_NAME}/dbcppp/src/*.cpp)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/control.cpp
  src/controller/stanley.cpp
  src/canprotocol/can_protocol.cpp
  ${CANProtocol}
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  nav_msgs 
  tf2 
  tf2_geometry_msgs 
  geometry_msgs 
  example_interfaces
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}
  ${Boost}
)