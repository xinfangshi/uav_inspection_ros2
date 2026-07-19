#pragma once
#include "behaviortree_cpp/action_node.h"
#include <rclcpp/rclcpp.hpp>

namespace uav_control {
namespace behavior_trees {

class HoverInspectAction : public BT::StatefulActionNode {
public:
    HoverInspectAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    rclcpp::Node::SharedPtr node_;
    int hover_counter_;
};

}
}