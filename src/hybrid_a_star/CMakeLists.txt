cmake_minimum_required(VERSION 3.5)
project(hybrid_a_star)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

include_directories(
  include/${PROJECT_NAME}
  include/${PROJECT_NAME}/hybridastar
)

FILE(GLOB NavSources include/${PROJECT_NAME}/hybridastar/*.cpp)

add_executable(${PROJECT_NAME}
  src/path_planning.cpp
  src/map/map.cpp
  src/planner/planner.cpp
  src/decision_making/decision_making.cpp
  src/main.cpp
  ${NavSources}
)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs vision_msgs geometry_msgs nav_msgs tf2 tf2_geometry_msgs example_interfaces)

target_link_libraries(${PROJECT_NAME}
  ompl
  Eigen3::Eigen
  ${CERES_LIBRARIES}
)

# Install executables
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
