cmake_minimum_required(VERSION 3.8)
project(coug_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(generate_parameter_library REQUIRED)

# --- Parameters ---
generate_parameter_library(
  waypoint_follower_parameters
  "params/waypoint_follower_params.yaml"
)

# --- Component Library ---
ament_auto_add_library(waypoint_follower_component SHARED src/waypoint_follower_node.cpp)
target_include_directories(waypoint_follower_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(waypoint_follower_component
  waypoint_follower_parameters
)

rclcpp_components_register_nodes(waypoint_follower_component
  "coug_navigation::WaypointFollowerNode"
)

# --- Executables ---
ament_auto_add_executable(waypoint_follower src/waypoint_follower_main.cpp)
target_link_libraries(waypoint_follower
  waypoint_follower_component
)

# --- Testing ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_waypoint_follower_node test/test_waypoint_follower_node.cpp)
  target_include_directories(test_waypoint_follower_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_waypoint_follower_node
    waypoint_follower_component
  )
  ament_target_dependencies(test_waypoint_follower_node
    rclcpp
    nav_msgs
    geometry_msgs
    std_msgs
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  USE_SCOPED_HEADER_INSTALL_DIR
)
