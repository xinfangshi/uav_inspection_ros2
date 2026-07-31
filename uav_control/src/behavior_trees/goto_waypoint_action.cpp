#include "uav_control/behavior_trees/goto_waypoint_action.hpp"
#include <algorithm> // 引入算法库用于边界限制

namespace uav_control
{
namespace behavior_trees
{

// 💡 修复 1：在构造函数中初始化 trajectory_generated_ 标志位
GoToWaypointAction::GoToWaypointAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node), odom_received_(false), trajectory_generated_(false)
{
    offboard_control_mode_publisher_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    
    odom_subscriber_ = node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
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
    // 💡 修复 2：每次节点被激活，重置轨迹生成标志位
    trajectory_generated_ = false; 
    RCLCPP_INFO(node_->get_logger(), "[GoToWaypoint] 🛸 收到航点任务，准备进行安全校验与轨迹生成...");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToWaypointAction::onRunning()
{
    // 闭环基石：没收到里程计绝对不乱动
    if (!odom_received_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[GoToWaypoint] ⏳ 等待里程计初始位置...");
        return BT::NodeStatus::RUNNING;
    }

    // 💡 修复 3：将庞大的安全校验和轨迹生成移入 onRunning，确保绝对会被执行！
    if (!trajectory_generated_) {
        double target_x = 0.0, target_y = 0.0, target_z = 0.0;
        getInput("x", target_x); getInput("y", target_y); getInput("z", target_z);

        // =========================================================================
        // 🛡️ 安全防线 1：高度电子围栏 (Altitude Geofence)
        // 绝对不允许飞机钻地 (Z > -0.5) 或者飞出平流层 (Z < -20.0)！
        // 注意：PX4 的 NED 坐标系下，Z 轴为负数代表在空中。
        // =========================================================================
        if (target_z > -0.5) {
            RCLCPP_WARN(node_->get_logger(), "🚨 安全拦截：目标高度太低 (%.2fm) 会导致坠机！强制拉升至 -1.0m", target_z);
            target_z = -1.0;
        } else if (target_z < -30.0) {
            RCLCPP_WARN(node_->get_logger(), "🚨 安全拦截：目标高度太高 (%.2fm) 超出空域！强制限制为 -30.0m", target_z);
            target_z = -30.0;
        }

        // =========================================================================
        // 🛡️ 安全防线 2：绝对物理边界限制 (X/Y Geofence)
        // 限制无人机只能在 X [-50, 50], Y [-50, 50] 的安全测试区域内飞行
        // =========================================================================
        target_x = std::clamp(target_x, -50.0, 50.0);
        target_y = std::clamp(target_y, -50.0, 50.0);

        Waypoint start_pt = {current_x_, current_y_, current_z_};
        Waypoint end_pt = {target_x, target_y, target_z};

        double distance = std::sqrt(std::pow(end_pt.x - start_pt.x, 2) + std::pow(end_pt.y - start_pt.y, 2) + std::pow(end_pt.z - start_pt.z, 2));
        
     // =========================================================================
        // 🛡️ 安全防线 3：动态动力学限速 (Dynamic Kinematic Constraints) 升级版！
        // =========================================================================
        // 降低期望平均速度至 0.8m/s，并强制基础缓冲时间提高到 4.0秒！
        // 在 Minimum Snap 中，时间 T 越长，多项式生成的加速度峰值就会呈几何级数下降！
        double T = std::max(4.0, distance / 0.8); 

        RCLCPP_INFO(node_->get_logger(), "🛡️ [安全规划] 强制生成柔和版 Minimum Snap 轨迹! 距离: %.2fm, 耗时: %.1fs", distance, T);

        current_trajectory_ = planner_.generate_trajectory(start_pt, end_pt, T, 0.02);
        trajectory_index_ = 0;
        
        // 标记轨迹已生成，下一帧进来就不再重新生成了！
        trajectory_generated_ = true; 
    }

    // --- 沿着轨迹点执行 ---
    // 如果还没飞完，继续发送轨迹点
    if (trajectory_index_ < current_trajectory_.size()) {
        Waypoint pt = current_trajectory_[trajectory_index_];
        
        publish_offboard_control_mode();
        publish_trajectory_setpoint(pt.x, pt.y, pt.z);
        
        trajectory_index_++; 
        return BT::NodeStatus::RUNNING;
    }

    // =========================================================================
    // 🛡️ 安全防线 4：防震荡软着陆 (Anti-Oscillation Hold)
    // 轨迹走完后，不要立刻退出。持续发送最后的终点坐标，让底层的 PID 彻底稳住机身。
    // =========================================================================
    double target_x = 0.0, target_y = 0.0, target_z = 0.0, acceptable_radius = 0.5;
    getInput("x", target_x); getInput("y", target_y); getInput("z", target_z); getInput("acceptable_radius", acceptable_radius);
    
    double distance = std::sqrt(std::pow(current_x_ - target_x, 2) + std::pow(current_y_ - target_y, 2) + std::pow(current_z_ - target_z, 2));

    publish_offboard_control_mode();
    publish_trajectory_setpoint(target_x, target_y, target_z);

    if (distance < acceptable_radius) {
        RCLCPP_INFO_ONCE(node_->get_logger(), "🎯 [GoToWaypoint] 姿态已稳定！误差: %.2f 米", distance);
        return BT::NodeStatus::SUCCESS;
    } else {
        // 如果被风吹偏了，慢慢吸回来
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