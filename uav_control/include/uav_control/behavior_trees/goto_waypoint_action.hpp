#pragma once

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/action_node.h"
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <cmath>
#include <atomic>

namespace uav_control
{
namespace behavior_trees
{

class GoToWaypointAction : public BT::StatefulActionNode
{
public:
    GoToWaypointAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(float x, float y, float z);
    
    // 回调函数：接收无人机当前位置
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    
    // 核心：新增订阅者，用来“看”自己的位置
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_subscriber_;
    
    // 使用原子操作保证多线程数据安全
    std::atomic<float> current_x_;
    std::atomic<float> current_y_;
    std::atomic<float> current_z_;
};

} // namespace behavior_trees
} // namespace uav_control