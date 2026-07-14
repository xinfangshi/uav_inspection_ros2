#pragma once

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/action_node.h"
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

namespace uav_control {
namespace behavior_trees {

// 注意这里！已经升级为 StatefulActionNode 啦！
class TakeoffAction : public BT::StatefulActionNode {
   public:
    // 构造函数新增了 rclcpp::Node::SharedPtr node
    TakeoffAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    // 必须实现的三个生命周期函数
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

   private:
    void arm();
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(float altitude);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    int setpoint_counter_;
};

}  // namespace behavior_trees
}  // namespace uav_control