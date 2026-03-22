#include "rclcpp/rclcpp.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"

#include "amr-bt/send_nav2_goal_action_node.hpp"
#include "amr-bt/rotate_action_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("amr_bt_node");

  BT::BehaviorTreeFactory factory;

  BT::RosNodeParams params;
  params.nh = node;
  params.wait_for_server_timeout = std::chrono::milliseconds(10000); // time to wait for server at startup

  factory.registerNodeType<SendWaypointAction>("SendWaypointAction", params);
  // RotateAction takes node handle directly
  factory.registerBuilder<RotateAction>(
      "RotateAction",
      [node](const std::string &name, const BT::NodeConfig &conf)
      {
        return std::make_unique<RotateAction>(name, conf, node);
      });

  // Load your XML tree
  auto tree = factory.createTreeFromFile("/home/kronton/ros2_ws/src/AMR-BT/amr-bt/behavior_trees/tree.xml");

  // set action_name on the blackboard before ticking
  // tree.rootBlackboard()->set("action_name", std::string("navigate_to_pose"));

  // Tick until done
  tree.tickWhileRunning();

  rclcpp::shutdown();
  return 0;
}