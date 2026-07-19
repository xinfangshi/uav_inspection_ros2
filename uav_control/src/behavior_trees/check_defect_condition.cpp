#include "uav_control/behavior_trees/check_defect_condition.hpp"
#include <iostream>

namespace uav_control {
namespace behavior_trees {

CheckDefectCondition::CheckDefectCondition(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList CheckDefectCondition::providedPorts() {
    return { 
        BT::InputPort<bool>("is_detected"),
        BT::InputPort<double>("target_x"),
        BT::InputPort<double>("target_y")
    };
}

BT::NodeStatus CheckDefectCondition::tick() {
    bool detected = false;
    double x = 0.0, y = 0.0;
    
    // 从黑板读取数据
    if (getInput("is_detected", detected) && detected) {
        getInput("target_x", x);
        getInput("target_y", y);
        std::cout << "\033[1;31m[Condition] 🚨 警报！雷达发现目标！像素坐标: (" << x << ", " << y << ")\033[0m\n";
        return BT::NodeStatus::SUCCESS; // 发现目标！放行！
    }
    return BT::NodeStatus::FAILURE; // 没发现，拦截！
}

}
}