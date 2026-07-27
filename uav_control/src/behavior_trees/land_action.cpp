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
    land_x_ = current_x_;
    land_y_ = current_y_;
    grounded_counter_ = 0; // 初始化接地计时器
    RCLCPP_INFO(node_->get_logger(), "[Land] 🛬 开始垂直降落...");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LandAction::onRunning() {
    // 必须持续发送心跳，否则飞控会立刻失控！
    publish_offboard_control_mode();
    publish_trajectory_setpoint(land_x_, land_y_, 1.0);

    float current_alt = -current_z_; 
    
    // 闭环判断：如果高度小于 0.25 米，开始计时！
    if (current_alt < 0.25) {
        grounded_counter_++;
        
        // 50Hz * 100 = 2秒。把飞机死死按在地上 3 秒钟，让飞控着陆检测器彻底确认！
        if (grounded_counter_ == 250) {
            RCLCPP_INFO(node_->get_logger(), "[Land] 🌍 接触地面并保持稳定！发送锁定电机 (Disarm) 指令...");
            disarm(); 
        }
        
        // 再等 1 秒钟，确保飞控执行了锁定指令，然后才安全退出程序！
        if (grounded_counter_ > 150) {
            RCLCPP_INFO(node_->get_logger(), "[Land] ✅ 降落与锁定彻底完毕，巡检任务圆满闭环！");
            return BT::NodeStatus::SUCCESS;
        }
    } else {
        // 如果中间被风吹起来了，计时器清零，重新算！
        grounded_counter_ = 0;
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