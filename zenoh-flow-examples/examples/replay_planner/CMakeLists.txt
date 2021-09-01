cmake_minimum_required(VERSION 3.5)
project(replay_planner)

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

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(autoware_auto_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)
ament_auto_find_build_dependencies()
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/replay_planner_wrapper.cpp
  src/recordreplay_planner.cpp
  src/msgs/autoware_auto_msgs/convert.cpp
  src/msgs/builtin_interfaces/convert.cpp
  src/msgs/geometry_msgs/convert.cpp
  src/msgs/std_msgs/convert.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp tf2_geometry_msgs)

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
