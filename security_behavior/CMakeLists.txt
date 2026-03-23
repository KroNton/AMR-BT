cmake_minimum_required(VERSION 3.8)
project(security_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
#  removed: find_package(amr-bt REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(amr_bt_main src/main.cpp)

ament_target_dependencies(amr_bt_main
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
  ament_index_cpp
)

# removed entire target_link_libraries block
# plugins are loaded at runtime — no linking needed

install(TARGETS amr_bt_main
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()