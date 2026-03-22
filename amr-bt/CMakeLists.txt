cmake_minimum_required(VERSION 3.8)
project(amr-bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(nav2_send_waypoint_client
  src/nav2_send_waypoint_client.cpp
)


ament_target_dependencies(nav2_send_waypoint_client
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

add_library(send_nav2_goal_action_node SHARED
  src/send_nav2_goal_action_node.cpp
)

ament_target_dependencies(send_nav2_goal_action_node
  rclcpp
  rclcpp_action
  behaviortree_cpp
  behaviortree_ros2
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS 
  nav2_send_waypoint_client
  DESTINATION lib/${PROJECT_NAME}
)


target_include_directories(send_nav2_goal_action_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


add_library(rotate_action_node SHARED
  src/rotate_action_node.cpp
)

ament_target_dependencies(rotate_action_node
  rclcpp
  rclcpp_action
  behaviortree_cpp
  behaviortree_ros2
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

target_include_directories(rotate_action_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS 
  send_nav2_goal_action_node
  rotate_action_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(amr_bt_main src/main.cpp)

ament_target_dependencies(amr_bt_main
  rclcpp
  behaviortree_ros2
)

target_link_libraries(amr_bt_main
  send_nav2_goal_action_node 
  rotate_action_node
)

install(TARGETS amr_bt_main
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
