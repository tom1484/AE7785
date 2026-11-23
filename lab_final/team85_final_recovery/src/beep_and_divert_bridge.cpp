#include "team85_final_recovery/beep_and_divert_bridge.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace team85_final_recovery
{

BeepAndDivertBridge::BeepAndDivertBridge(
  const std::string & name,
  const BT::NodeConfiguration & config)
: nav2_behavior_tree::BtServiceNode<ServiceType>(
    name,
    config,
    "/divert_robot")
{
}

void BeepAndDivertBridge::on_tick()
{
  RCLCPP_INFO(
    node_->get_logger(),
    "BeepAndDivertBridge: Calling recovery service..."
  );
}

BT::NodeStatus BeepAndDivertBridge::on_completion(
  std::shared_ptr<ServiceType::Response> response)
{
  if (!response->success) {
    RCLCPP_WARN(
      node_->get_logger(),
      "BeepAndDivertBridge: Recovery service failed"
    );

    return BT::NodeStatus::FAILURE;
  }

  const auto & pose = response->recovery_pose;

  auto result = setOutput("recovery_pose", pose);

  if (!result) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "BeepAndDivertBridge: Failed to write recovery_pose"
    );

    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "BeepAndDivertBridge: Recovery pose received: (%.2f, %.2f)",
    pose.pose.position.x,
    pose.pose.position.y
  );

  return BT::NodeStatus::SUCCESS;
}

}  // namespace team85_final_recovery


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<
    team85_final_recovery::BeepAndDivertBridge>(
      "BeepAndDivertBridge");
}