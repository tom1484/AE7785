#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "team85_final_recovery/srv/divert_robot.hpp"

namespace team85_final_recovery
{

class BeepAndDivertBridge
  : public nav2_behavior_tree::BtServiceNode<
      team85_final_recovery::srv::DivertRobot>
{
public:
  using ServiceType = team85_final_recovery::srv::DivertRobot;

  BeepAndDivertBridge(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "recovery_pose",
        "Diversion waypoint returned by the recovery service")
    });
  }

  void on_tick() override;

  BT::NodeStatus on_completion(
    std::shared_ptr<ServiceType::Response> response) override;
};

}  // namespace team85_final_recovery