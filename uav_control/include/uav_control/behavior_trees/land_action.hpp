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

class LandAction : public BT::StatefulActionNode {
public:
    LandAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void disarm();
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(float x, float y, float z);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_subscriber_;
    
    std::atomic<float> current_x_;
    std::atomic<float> current_y_;
    std::atomic<float> current_z_;
    std::atomic<bool> odom_received_;
    
    // 降落时锁死平面的坐标
    float land_x_;
    float land_y_;
    // 👇 新增：接地计时器
    int grounded_counter_;
};

} // namespace behavior_trees
} // namespace uav_control