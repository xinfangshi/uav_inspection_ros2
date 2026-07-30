#pragma once

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/action_node.h"
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <atomic>
#include <cmath>

namespace uav_control {
namespace behavior_trees {

class HoverInspectAction : public BT::StatefulActionNode {
public:
    HoverInspectAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(float x, float y, float z, float yaw);
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    // 计算四元数到欧拉角(Yaw)的转换
    double get_yaw_from_quaternion(float w, float x, float y, float z);

    rclcpp::Node::SharedPtr node_;
    
    // 🔥 新增：控制无人机悬停与旋转的发布者
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    
    // 🔥 新增：监听自身状态的雷达
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_subscriber_;

    // 状态数据
    std::atomic<bool> odom_received_;
    std::atomic<float> current_x_, current_y_, current_z_;
    std::atomic<float> current_vx_, current_vy_; // 当前速度
    std::atomic<float> current_yaw_;             // 当前偏航角

    // 锁定的悬停目标与对焦角度
    float hover_x_, hover_y_, hover_z_;
    float target_yaw_;

    int hover_counter_;
    bool is_stable_; // 姿态是否平稳的标志
};

}
}