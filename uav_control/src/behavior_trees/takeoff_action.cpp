#include "uav_control/behavior_trees/takeoff_action.hpp"

namespace uav_control {
namespace behavior_trees {

TakeoffAction::TakeoffAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node), current_z_(0.0), odom_received_(false), armed_(false) {
    offboard_control_mode_publisher_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    odom_subscriber_ = node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(), 
        std::bind(&TakeoffAction::odom_callback, this, std::placeholders::_1));
}

BT::PortsList TakeoffAction::providedPorts() { return {BT::InputPort<double>("target_altitude")}; }

void TakeoffAction::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    current_x_ = msg->position[0]; // 新增读取当前 XY
    current_y_ = msg->position[1];
    current_z_ = msg->position[2]; 
    odom_received_ = true;
}

BT::NodeStatus TakeoffAction::onStart() {
    setpoint_counter_ = 0; armed_ = false;
    RCLCPP_INFO(node_->get_logger(), "[Takeoff] ✈️ 起飞节点激活，等待里程计信号...");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TakeoffAction::onRunning() {
    if (!odom_received_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[Takeoff] ⏳ 致命错误：等待里程计...");
        return BT::NodeStatus::RUNNING;
    }

    // 🔥 核心修复 1：在准备发指令的第 0 帧，死死锁住当前的 XY 坐标！
    if (setpoint_counter_ == 0) {
        start_x_ = current_x_;
        start_y_ = current_y_;
    }

    double altitude = 5.0;
    getInput("target_altitude", altitude);

    publish_offboard_control_mode();
    // 🔥 发送指令时，X 和 Y 保持不动，只让 Z 上升！这才是真正的垂直起飞！
    publish_trajectory_setpoint(start_x_, start_y_, -altitude);

    if (setpoint_counter_ == 50 && !armed_) {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        arm();
        armed_ = true;
    }
    if (setpoint_counter_ < 100) setpoint_counter_++;

    float current_alt = -current_z_; 
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
    msg.position = true; msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

// 注意这里修改了参数，接收 x, y, z
void TakeoffAction::publish_trajectory_setpoint(float x, float y, float z) {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {x, y, z}; msg.yaw = 0.0; msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
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