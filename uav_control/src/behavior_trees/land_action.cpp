#include "uav_control/behavior_trees/land_action.hpp"

namespace uav_control {
namespace behavior_trees {

LandAction::LandAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node), odom_received_(false) {
    
    offboard_control_mode_publisher_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    
    odom_subscriber_ = node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(), 
        std::bind(&LandAction::odom_callback, this, std::placeholders::_1));
}

BT::PortsList LandAction::providedPorts() { return {}; } // 降落不需要传参，直接落

void LandAction::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    current_x_ = msg->position[0];
    current_y_ = msg->position[1];
    current_z_ = msg->position[2]; 
    odom_received_ = true;
}

BT::NodeStatus LandAction::onStart() {
    if (!odom_received_) {
        RCLCPP_WARN(node_->get_logger(), "[Land] ⏳ 等待里程计...");
        return BT::NodeStatus::RUNNING;
    }
    
    // 🔥 修复 Bug 3：不再发送 Offboard 位置，而是下发 PX4 原生自动降落指令 (Command ID: 21)
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(node_->get_logger(), "[Land] 🛬 切换至 PX4 原生自动降落模式 (AUTO.LAND)...");
    
    // 初始化防抖计时器
    land_x_ = current_z_; // 借用这个变量存上一次的高度
    grounded_counter_ = 0; 
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LandAction::onRunning() {
    // PX4 正在自动降落，我们只需要在旁边监控高度变化即可！
    
    // 计算当前高度和上一帧高度的差值
    float delta_z = std::abs(current_z_ - land_x_);
    land_x_ = current_z_;

    // 如果高度几乎不怎么变化了 (垂直速度接近 0)
    if (delta_z < 0.005) {
        grounded_counter_++;
    } else {
        grounded_counter_ = 0; // 如果还在下降，计时器清零
    }

    // 监控 3 秒钟 (50Hz * 3s = 150)
    if (grounded_counter_ > 150) {
        RCLCPP_INFO(node_->get_logger(), "[Land] ✅ 降落彻底完毕！PX4 将自动执行锁定电机。任务圆满闭环！");
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::RUNNING;
}

void LandAction::onHalted() { RCLCPP_WARN(node_->get_logger(), "[Land] ⚠️ 降落被中断！"); }

void LandAction::disarm() {
    // 发送 Disarm 指令 (param1 = 0.0 表示锁定，1.0 表示解锁)
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}

void LandAction::publish_offboard_control_mode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true; msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void LandAction::publish_trajectory_setpoint(float x, float y, float z) {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {x, y, z}; msg.yaw = 0.0; msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void LandAction::publish_vehicle_command(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1; msg.param2 = param2; msg.command = command;
    msg.target_system = 1; msg.target_component = 1; msg.source_system = 1; msg.source_component = 1;
    msg.from_external = true; msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

} 
}