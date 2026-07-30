#include "uav_control/behavior_trees/hover_inspect_action.hpp"

namespace uav_control {
namespace behavior_trees {

HoverInspectAction::HoverInspectAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node), odom_received_(false) {
    
    offboard_control_mode_publisher_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    
    odom_subscriber_ = node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(), 
        std::bind(&HoverInspectAction::odom_callback, this, std::placeholders::_1));
}

BT::PortsList HoverInspectAction::providedPorts() { return {}; }

void HoverInspectAction::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    current_x_ = msg->position[0];
    current_y_ = msg->position[1];
    current_z_ = msg->position[2];
    current_vx_ = msg->velocity[0];
    current_vy_ = msg->velocity[1];
    current_yaw_ = get_yaw_from_quaternion(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    odom_received_ = true;
}

double HoverInspectAction::get_yaw_from_quaternion(float w, float x, float y, float z) {
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

BT::NodeStatus HoverInspectAction::onStart() {
    if (!odom_received_) {
        RCLCPP_WARN(node_->get_logger(), "[HoverInspect] ⏳ 等待里程计...");
        return BT::NodeStatus::RUNNING;
    }

    // 1. 紧急刹车！把刚才被打断那一瞬间的坐标，作为悬停死锚点！
    hover_x_ = current_x_;
    hover_y_ = current_y_;
    hover_z_ = current_z_;

    // 2. 从黑板读取刚才 AI 发现的目标绝对物理坐标
    double target_x = 0, target_y = 0;
    if (config().blackboard->get<double>("defect_x", target_x) && config().blackboard->get<double>("defect_y", target_y)) {
        // 🔥 核心几何计算：计算目标相对于无人机的方位角 (Yaw)
        // 这样不管飞机怎么飞，只要打断，机头一定会旋转去死死对准目标！
        target_yaw_ = std::atan2(target_y - hover_y_, target_x - hover_x_);
    } else {
        target_yaw_ = current_yaw_; // 没读到就保持原样
    }

    hover_counter_ = 0;
    is_stable_ = false;
    RCLCPP_INFO(node_->get_logger(), "📸 [视觉伺服] 打断巡航！紧急刹车并旋转机头对准目标 (目标偏航角: %.2f rad)...", target_yaw_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoverInspectAction::onRunning() {
    // 持续发送 Offboard 指令，命令飞控死死钉在悬停点，并且旋转机头！
    publish_offboard_control_mode();
    publish_trajectory_setpoint(hover_x_, hover_y_, hover_z_, target_yaw_);

    // === 姿态平稳判定机制 ===
    if (!is_stable_) {
        // 计算平面速度大小 (评估刹车是否停稳)
        float speed = std::sqrt(current_vx_ * current_vx_ + current_vy_ * current_vy_);
        // 计算偏航角误差 (评估机头是否对准)
        float yaw_error = std::abs(target_yaw_ - current_yaw_);
        // 消除 360 度回环误差
        if (yaw_error > M_PI) yaw_error = 2 * M_PI - yaw_error;

        // 如果速度降至 0.2m/s 以下，且对准误差小于 0.1 弧度(约5度)，判定为平稳！
        if (speed < 0.2f && yaw_error < 0.1f) {
            is_stable_ = true;
            RCLCPP_INFO(node_->get_logger(), "🎯 [视觉伺服] 姿态已彻底平稳！目标已锁定在画面正中心，开始 3 秒高清曝光...");
        } else {
            // 还在晃动，继续等
            return BT::NodeStatus::RUNNING;
        }
    }

    // === 拍照倒计时 ===
    if (is_stable_) {
        hover_counter_++;
        if (hover_counter_ > 150) { // 50Hz * 3s = 150
            RCLCPP_INFO(node_->get_logger(), "✅ [HoverInspect] 拍摄完成！目标特征已保存。");
            
            // 写入黑板，触发大管家的“空间记忆黑名单”机制
            config().blackboard->set<bool>("just_inspected", true);
            
            return BT::NodeStatus::SUCCESS;
        }
    }

    return BT::NodeStatus::RUNNING;
}

void HoverInspectAction::onHalted() {
    RCLCPP_WARN(node_->get_logger(), "[HoverInspect] ⚠️ 悬停拍摄被意外中断！");
}

void HoverInspectAction::publish_offboard_control_mode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void HoverInspectAction::publish_trajectory_setpoint(float x, float y, float z, float yaw) {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {x, y, z};
    msg.yaw = yaw; // 下发目标偏航角！
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

}
}