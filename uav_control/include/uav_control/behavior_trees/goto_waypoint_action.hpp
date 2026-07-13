#pragma once

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/action_node.h"
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <cmath>
#include <atomic>
#include <vector>
// 引入轨迹规划器
#include "uav_control/planners/trajectory_planner.hpp"

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
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_subscriber_;
    
    std::atomic<float> current_x_;
    std::atomic<float> current_y_;
    std::atomic<float> current_z_;
    std::atomic<bool> odom_received_; 

    // ========== 轨迹追踪变量 ==========
    TrajectoryPlanner planner_;                  // 规划器实例
    std::vector<Waypoint> current_trajectory_;   // 存放算出来的几百个密集坐标点
    size_t trajectory_index_;                    // 当前飞到了第几个点
};

} // namespace behavior_trees
} // namespace uav_control