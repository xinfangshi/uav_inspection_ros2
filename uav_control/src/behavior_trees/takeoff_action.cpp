#include "uav_control/behavior_trees/takeoff_action.hpp"

namespace uav_control {
namespace behavior_trees {

// 构造函数：接收 node 并初始化 Publishers
TakeoffAction::TakeoffAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node) {
    // 利用传入的 node_ 指针，创建话题发布者
    offboard_control_mode_publisher_ =
        node_->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ =
        node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
}

BT::PortsList TakeoffAction::providedPorts() { return {BT::InputPort<double>("target_altitude")}; }

// 行为树第一次执行到该节点时调用
BT::NodeStatus TakeoffAction::onStart() {
    setpoint_counter_ = 0;
    RCLCPP_INFO(node_->get_logger(), "[Takeoff] ✈️ 起飞节点激活，开始预热 Offboard 信号...");
    return BT::NodeStatus::RUNNING;  // 告诉行为树：我还没干完，下次继续叫我 (Tick)
}

// 行为树持续 Tick 时调用（相当于原来的 timer_callback）
BT::NodeStatus TakeoffAction::onRunning() {
    double altitude = 5.0;  // 默认 5 米
    getInput("target_altitude", altitude);

    // 1. 发送心跳和目标高度 (NED 坐标系，高度向下为正，所以乘 -1)
    publish_offboard_control_mode();
    publish_trajectory_setpoint(-altitude);

    // 2. 预热 50 次 (如果执行器频率是 50Hz，就是 1 秒) 后解锁起飞
    if (setpoint_counter_ == 50) {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        arm();
    }

    setpoint_counter_++;

    // 3. 模拟起飞过程，假设发了 200 次指令（大概 4 秒），我们认为它飞到了
    // (真正完美的做法是订阅里程计，判断实际高度是否达到 altitude，我们后续再优化)
    if (setpoint_counter_ > 200) {
        RCLCPP_INFO(node_->get_logger(), "[Takeoff] ✅ 起飞完成，到达指定高度！");
        return BT::NodeStatus::SUCCESS;  // 告诉行为树：我干完了，你可以执行下一个节点了！
    }

    return BT::NodeStatus::RUNNING;  // 还没到 200 次，继续保持 RUNNING
}

void TakeoffAction::onHalted() { RCLCPP_WARN(node_->get_logger(), "[Takeoff] ⚠️ 起飞被强行中断！"); }

void TakeoffAction::arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(node_->get_logger(), "[Takeoff] ⚔️ 发送解锁(Arm)指令!");
}

void TakeoffAction::publish_offboard_control_mode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void TakeoffAction::publish_trajectory_setpoint(float altitude) {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {0.0, 0.0, altitude};
    msg.yaw = 0.0;
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void TakeoffAction::publish_vehicle_command(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

}  // namespace behavior_trees
}  // namespace uav_control