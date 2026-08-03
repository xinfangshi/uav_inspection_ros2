#pragma once

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/action_node.h"
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <atomic>
#include <cmath>

#include <geometry_msgs/msg/point.hpp>

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
    // 🔥 架构升级：加入控制模式标志位 (速度控制 vs 位置控制)
    void publish_offboard_control_mode(bool position_control, bool velocity_control);
    void publish_trajectory_setpoint(float x, float y, float z, float yaw, bool is_velocity_mode);
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    double get_yaw_from_quaternion(float w, float x, float y, float z);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_subscriber_;

    // 新增：用于广播“已完成巡检目标坐标”的喇叭
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr inspected_pub_;

    std::atomic<bool> odom_received_;
    std::atomic<float> current_x_, current_y_, current_z_;
    std::atomic<float> current_vx_, current_vy_; 
    std::atomic<float> current_yaw_;             

    float hover_x_, hover_y_, hover_z_;
    float target_yaw_;

    // 🔥 三段式状态机变量
    enum HoverState { BRAKING, ALIGNING, SHOOTING };
    HoverState current_state_;
    
    int stable_hold_counter_; // 连续稳定帧计数器 (消除钟摆效应)
    int hover_counter_;       // 拍照倒计时

    float brake_yaw_; 
    
    // 🔥 新增：用于死死锁住警报触发瞬间的目标真实坐标！
    double locked_defect_x_, locked_defect_y_, locked_defect_z_;

};

}
}