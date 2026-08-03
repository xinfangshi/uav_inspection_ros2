#pragma once

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/action_node.h"
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <atomic>

namespace uav_control {
namespace behavior_trees {

class TakeoffAction : public BT::StatefulActionNode {
public:
    TakeoffAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void arm();
    void publish_offboard_control_mode();
    
    // 🔥 修复点 1：签名同步为接收 x, y, z 三个参数！
    void publish_trajectory_setpoint(float x, float y, float z); 
    
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_subscriber_;
    
    uint64_t setpoint_counter_;
    
    // 🔥 修复点 2：补齐所有 X, Y, Z 里程计状态变量！
    std::atomic<float> current_x_;
    std::atomic<float> current_y_;
    std::atomic<float> current_z_;
    std::atomic<bool> odom_received_;
    
    // 🔥 修复点 3：起飞防偏航锁死变量！
    bool armed_;
    float start_x_;
    float start_y_;

    int settle_counter_;
};

} // namespace behavior_trees
} // namespace uav_control