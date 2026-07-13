#include "uav_control/behavior_trees/goto_waypoint_action.hpp"

namespace uav_control
{
namespace behavior_trees
{

GoToWaypointAction::GoToWaypointAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node), odom_received_(false)
{
    offboard_control_mode_publisher_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    
    odom_subscriber_ = node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
        std::bind(&GoToWaypointAction::odom_callback, this, std::placeholders::_1));

    current_x_ = 0.0; current_y_ = 0.0; current_z_ = 0.0;
}

BT::PortsList GoToWaypointAction::providedPorts()
{
    return { 
        BT::InputPort<double>("x"), BT::InputPort<double>("y"), BT::InputPort<double>("z"),
        BT::InputPort<double>("acceptable_radius") 
    };
}

void GoToWaypointAction::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    current_x_ = msg->position[0];
    current_y_ = msg->position[1];
    current_z_ = msg->position[2];
    odom_received_ = true;
}

BT::NodeStatus GoToWaypointAction::onStart()
{
    if (!odom_received_) {
        RCLCPP_WARN(node_->get_logger(), "[GoToWaypoint] 等待里程计初始位置...");
        return BT::NodeStatus::RUNNING;
    }

    double target_x = 0.0, target_y = 0.0, target_z = 0.0;
    getInput("x", target_x); getInput("y", target_y); getInput("z", target_z);

    // 1. 获取起点和终点
    Waypoint start_pt = {current_x_, current_y_, current_z_};
    Waypoint end_pt = {target_x, target_y, target_z};

    // 2. 根据距离，估算飞行总时间 T (假设期望平均速度为 2 m/s)
    double distance = std::sqrt(std::pow(end_pt.x - start_pt.x, 2) + std::pow(end_pt.y - start_pt.y, 2) + std::pow(end_pt.z - start_pt.z, 2));
    double T = std::max(2.0, distance / 2.0); // 最少飞 2 秒，保证起降平稳

    RCLCPP_INFO(node_->get_logger(), "[GoToWaypoint] 🛸 开始生成 Minimum Snap 轨迹! 距离: %.2fm, 预计耗时: %.1fs", distance, T);

    // 3. 调用 OSQP 求解器，瞬间生成极度平滑的轨迹点！(步长 0.02s 对应我们执行器的 50Hz)
    current_trajectory_ = planner_.generate_trajectory(start_pt, end_pt, T, 0.02);
    trajectory_index_ = 0;

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToWaypointAction::onRunning()
{
    // 如果我们还在按着轨迹飞
    if (trajectory_index_ < current_trajectory_.size()) {
        Waypoint pt = current_trajectory_[trajectory_index_];
        
        publish_offboard_control_mode();
        publish_trajectory_setpoint(pt.x, pt.y, pt.z);
        
        trajectory_index_++; // 执行器 50Hz 滴答一次，我们就往下一个轨迹点走一步！
        return BT::NodeStatus::RUNNING;
    }

    // 轨迹走完了，进行最后的收尾检查：是否真实到达了目标半径内
    double target_x = 0.0, target_y = 0.0, target_z = 0.0, acceptable_radius = 0.5;
    getInput("x", target_x); getInput("y", target_y); getInput("z", target_z); getInput("acceptable_radius", acceptable_radius);
    
    double distance = std::sqrt(std::pow(current_x_ - target_x, 2) + std::pow(current_y_ - target_y, 2) + std::pow(current_z_ - target_z, 2));

    if (distance < acceptable_radius) {
        RCLCPP_INFO(node_->get_logger(), "[GoToWaypoint] 🎯 平滑降落！完美到达目标点! 误差: %.2f 米", distance);
        return BT::NodeStatus::SUCCESS;
    } else {
        // 万一有风吹偏了，保持发布终点，依靠 PX4 底层 PID 把它拉回来
        publish_offboard_control_mode();
        publish_trajectory_setpoint(target_x, target_y, target_z);
        return BT::NodeStatus::RUNNING;
    }
}

void GoToWaypointAction::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "[GoToWaypoint] ⚠️ 导航被强行中断！");
}

void GoToWaypointAction::publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void GoToWaypointAction::publish_trajectory_setpoint(float x, float y, float z)
{
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {x, y, z};
    msg.yaw = 0.0;
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

} // namespace behavior_trees
} // namespace uav_control