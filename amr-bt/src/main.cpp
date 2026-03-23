#include "rclcpp/rclcpp.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "ament_index_cpp/get_package_prefix.hpp"  // ✅ add this

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("amr_bt_node");

  BT::BehaviorTreeFactory factory;

  const std::string pkg_lib =
    ament_index_cpp::get_package_prefix("amr-bt") + "/lib/";

  factory.registerFromPlugin(pkg_lib + "libsend_nav2_goal_action_node.so");
  factory.registerFromPlugin(pkg_lib + "librotate_action_node.so");

  // ✅ create blackboard first and set values on it
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("action_name", std::string("navigate_to_pose"));

  // ✅ pass blackboard INTO createTreeFromFile — not after
  auto tree = factory.createTreeFromFile(
    "/home/kronton/ros2_ws/src/AMR-BT/amr-bt/behavior_trees/tree.xml",
    blackboard);  // ← blackboard passed here

  tree.tickWhileRunning();

  rclcpp::shutdown();
  return 0;
}