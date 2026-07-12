#include "uav_control/behavior_trees/goto_waypoint_action.hpp"

namespace uav_control
{
namespace behavior_trees
{

GoToWaypointAction::GoToWaypointAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    offboard_control_mode_publisher_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    
    // 初始化时订阅里程计话题
    odom_subscriber_ = node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
        std::bind(&GoToWaypointAction::odom_callback, this, std::placeholders::_1));

    current_x_ = 0.0;
    current_y_ = 0.0;
    current_z_ = 0.0;
}

BT::PortsList GoToWaypointAction::providedPorts()
{
    return { 
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y"),
        BT::InputPort<double>("z"),
        BT::InputPort<double>("acceptable_radius") 
    };
}

void GoToWaypointAction::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    // PX4 的里程计位置存在 position 数组中 [0]=X, [1]=Y, [2]=Z
    current_x_ = msg->position[0];
    current_y_ = msg->position[1];
    current_z_ = msg->position[2];
}

BT::NodeStatus GoToWaypointAction::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "[GoToWaypoint] 🛸 收到航点任务，开始前往目标...");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToWaypointAction::onRunning()
{
    double target_x = 0.0, target_y = 0.0, target_z = 0.0, acceptable_radius = 0.5;
    getInput("x", target_x);
    getInput("y", target_y);
    getInput("z", target_z);
    getInput("acceptable_radius", acceptable_radius);

    // 1. 发送移动指令
    publish_offboard_control_mode();
    publish_trajectory_setpoint(target_x, target_y, target_z);

    // 2. 闭环判断：计算当前位置与目标位置的 3D 欧式距离
    float dx = current_x_ - target_x;
    float dy = current_y_ - target_y;
    float dz = current_z_ - target_z;
    float distance = std::sqrt(dx*dx + dy*dy + dz*dz);

    // 3. 如果距离小于容忍半径，说明到达目标！
    if (distance < acceptable_radius) {
        RCLCPP_INFO(node_->get_logger(), "[GoToWaypoint] 🎯 成功到达目标点! 误差: %.2f 米", distance);
        return BT::NodeStatus::SUCCESS;
    }

    // 否则继续飞
    return BT::NodeStatus::RUNNING;
}

void GoToWaypointAction::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "[GoToWaypoint] ⚠️ 导航被强行中断！");
}

void GoToWaypointAction::publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true; // 我们依靠坐标位置导航
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void GoToWaypointAction::publish_trajectory_setpoint(float x, float y, float z)
{
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {x, y, z};
    msg.yaw = 0.0; // 航向角保持 0 (机头朝北)
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

} // namespace behavior_trees
} // namespace uav_control