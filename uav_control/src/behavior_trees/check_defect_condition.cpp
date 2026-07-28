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
        BT::InputPort<double>("target_y"),
        BT::InputPort<double>("target_z")   // 🔥 新增：接收 Z 轴高度！
    };
}

BT::NodeStatus CheckDefectCondition::tick() {
    bool detected = false;
    double x = 0.0, y = 0.0, z = 0.0;
    
    // 从黑板读取数据
    if (getInput("is_detected", detected) && detected) {
        getInput("target_x", x);
        getInput("target_y", y);
        getInput("target_z", z);  // 🔥 新增：读取 Z 轴高度！
        
        // 打印极具震撼感的全维 3D 物理坐标！
        RCLCPP_INFO(rclcpp::get_logger("Condition"), 
            "\033[1;31m🚨 警报！雷达锁定目标！绝对物理坐标: (X=%.2fm, Y=%.2fm, Z=%.2fm)\033[0m", x, y, z);
            
        return BT::NodeStatus::SUCCESS; 
    }
    return BT::NodeStatus::FAILURE; 
}

}
}