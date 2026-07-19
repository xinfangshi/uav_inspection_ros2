#include "uav_control/behavior_trees/hover_inspect_action.hpp"

namespace uav_control {
namespace behavior_trees {

HoverInspectAction::HoverInspectAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node) {}

BT::PortsList HoverInspectAction::providedPorts() { return {}; }

BT::NodeStatus HoverInspectAction::onStart() {
    hover_counter_ = 0;
    RCLCPP_INFO(node_->get_logger(), "[HoverInspect] 📸 巡航已被打断！进入悬停锁定模式，开始高清拍摄缺陷...");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoverInspectAction::onRunning() {
    hover_counter_++;
    // 模拟拍照驻留 3 秒 (50Hz * 3s = 150)
    if (hover_counter_ > 150) {
        RCLCPP_INFO(node_->get_logger(), "[HoverInspect] ✅ 拍摄完成！目标数据已保存。");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void HoverInspectAction::onHalted() {
    RCLCPP_WARN(node_->get_logger(), "[HoverInspect] ⚠️ 悬停拍摄被打断！");
}

}
}