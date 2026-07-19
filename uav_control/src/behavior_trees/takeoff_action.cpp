#include "uav_control/behavior_trees/takeoff_action.hpp"

namespace uav_control {
namespace behavior_trees {

TakeoffAction::TakeoffAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node), current_z_(0.0), odom_received_(false), armed_(false) {
    
    offboard_control_mode_publisher_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    
    // 注册订阅者，注意 QoS 采用 best_effort 完美匹配 PX4
    odom_subscriber_ = node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", 
        rclcpp::QoS(10).best_effort(), 
        std::bind(&TakeoffAction::odom_callback, this, std::placeholders::_1));
}

BT::PortsList TakeoffAction::providedPorts() { 
    return {BT::InputPort<double>("target_altitude")}; 
}

void TakeoffAction::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    current_z_ = msg->position[2]; // NED坐标系：Z向下为正，空中为负
    odom_received_ = true;
}

BT::NodeStatus TakeoffAction::onStart() {
    setpoint_counter_ = 0;
    armed_ = false;
    RCLCPP_INFO(node_->get_logger(), "[Takeoff] ✈️ 起飞节点激活，等待里程计信号...");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TakeoffAction::onRunning() {
    // 闭环第一关：防爆锁！收不到真实位置，绝对不起飞！
    if (!odom_received_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[Takeoff] ⏳ 致命错误：等待里程计初始位置中...");
        return BT::NodeStatus::RUNNING;
    }

    double altitude = 5.0;
    getInput("target_altitude", altitude);

    // 发送指令 (Z 为负数表示起飞)
    publish_offboard_control_mode();
    publish_trajectory_setpoint(-altitude);

    // 预热 1 秒后，执行一次性解锁
    if (setpoint_counter_ == 50 && !armed_) {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        arm();
        armed_ = true;
    }
    if (setpoint_counter_ < 100) setpoint_counter_++;

    // 闭环第二关：真实高度判定！
    float current_alt = -current_z_; // 把底层负数转成直观的正数高度
    // 只要距离目标高度误差小于 0.3 米，才算真正起飞成功！
    if (armed_ && std::abs(current_alt - altitude) < 0.3) {
        RCLCPP_INFO(node_->get_logger(), "[Takeoff] ✅ 起飞完成，实际到达高度: %.2f 米", current_alt);
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void TakeoffAction::onHalted() { RCLCPP_WARN(node_->get_logger(), "[Takeoff] ⚠️ 起飞被强行中断！"); }

void TakeoffAction::arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(node_->get_logger(), "[Takeoff] ⚔️ 发送解锁(Arm)指令!");
}

void TakeoffAction::publish_offboard_control_mode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
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
    msg.param1 = param1; msg.param2 = param2; msg.command = command;
    msg.target_system = 1; msg.target_component = 1; msg.source_system = 1; msg.source_component = 1;
    msg.from_external = true; msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

}
}