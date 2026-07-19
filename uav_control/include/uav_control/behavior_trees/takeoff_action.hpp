#pragma once

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/action_node.h"
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp> // 新增里程计头文件
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
    void publish_trajectory_setpoint(float altitude);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    // 监听高度的回调函数
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    
    // 新增：里程计订阅者与线程安全的坐标变量
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_subscriber_;
    uint64_t setpoint_counter_;
    std::atomic<float> current_z_;
    std::atomic<bool> odom_received_;
    bool armed_;
};

} // namespace behavior_trees
} // namespace uav_control