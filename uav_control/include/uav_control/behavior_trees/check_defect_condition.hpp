#pragma once
#include "behaviortree_cpp/condition_node.h"

namespace uav_control {
namespace behavior_trees {

class CheckDefectCondition : public BT::ConditionNode {
public:
    CheckDefectCondition(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

}
}