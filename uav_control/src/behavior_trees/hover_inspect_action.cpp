#include "uav_control/behavior_trees/hover_inspect_action.hpp"

namespace uav_control {
namespace behavior_trees {

HoverInspectAction::HoverInspectAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node), odom_received_(false) {
    offboard_control_mode_publisher_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

    // 🔥 引入极其稳定的 Transient Local 策略，确保 100% 必达！
    rclcpp::QoS inspected_qos(10);
    inspected_qos.transient_local();
    inspected_pub_ = node_->create_publisher<geometry_msgs::msg::Point>("/vision/inspected_target", inspected_qos);

    odom_subscriber_ = node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(), 
        std::bind(&HoverInspectAction::odom_callback, this, std::placeholders::_1));
}

BT::PortsList HoverInspectAction::providedPorts() { return {}; }

void HoverInspectAction::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    current_x_ = msg->position[0]; current_y_ = msg->position[1]; current_z_ = msg->position[2];
    current_vx_ = msg->velocity[0]; current_vy_ = msg->velocity[1];
    current_yaw_ = get_yaw_from_quaternion(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    odom_received_ = true;
}

double HoverInspectAction::get_yaw_from_quaternion(float w, float x, float y, float z) {
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

BT::NodeStatus HoverInspectAction::onStart() {
    if (!odom_received_) return BT::NodeStatus::RUNNING;

    brake_yaw_ = current_yaw_;
    
    // 🔥 核心修复 1A：在打断的第 0 毫秒，立刻读取并锁死目标坐标！
    if (config().blackboard->get<double>("defect_x", locked_defect_x_) && 
        config().blackboard->get<double>("defect_y", locked_defect_y_) &&
        config().blackboard->get<double>("defect_z", locked_defect_z_)) {
        
        double dist_2d = std::sqrt(std::pow(locked_defect_x_ - current_x_, 2) + std::pow(locked_defect_y_ - current_y_, 2));
        if (dist_2d < 2.5) {
            target_yaw_ = current_yaw_; 
        } else {
            target_yaw_ = std::atan2(locked_defect_y_ - current_y_, locked_defect_x_ - current_x_);
        }
    } else {
        target_yaw_ = current_yaw_;
    }

    current_state_ = BRAKING;
    stable_hold_counter_ = 0; hover_counter_ = 0;
    RCLCPP_INFO(node_->get_logger(), "📸 [视觉伺服] 触发！锁死目标坐标 (X:%.2f, Y:%.2f)，开始平滑刹车...", locked_defect_x_, locked_defect_y_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoverInspectAction::onRunning() {
    float speed = std::sqrt(current_vx_ * current_vx_ + current_vy_ * current_vy_);
    float yaw_error = std::abs(target_yaw_ - current_yaw_);
    if (yaw_error > M_PI) yaw_error = 2 * M_PI - yaw_error;

    // =========================================================
    // 🛡️ 阶段 1：柔性刹车 (Velocity Control)
    // =========================================================
    if (current_state_ == BRAKING) {
        publish_offboard_control_mode(false, true); // 关位置，开速度控制
        // 🔥 核心修复 2B：刹车时绝对不许转头！死死锁住 brake_yaw_！
        publish_trajectory_setpoint(0.0, 0.0, 0.0, brake_yaw_, true); 
        // 当物理速度降到 0.5m/s 以下，说明刹车基本完成，切入阶段 2
        if (speed < 0.5f) {
            hover_x_ = current_x_; hover_y_ = current_y_; hover_z_ = current_z_; // 此时再锁死坐标！
            current_state_ = ALIGNING;
            RCLCPP_INFO(node_->get_logger(), "📸 [视觉伺服] 刹车完毕！[阶段2] 锁定坐标，开始连续姿态稳定校验...");
        }
    }
    // =========================================================
    // 🛡️ 阶段 2：姿态稳定校验 (Position Control + Anti-Pendulum)
    // =========================================================
    else if (current_state_ == ALIGNING) {
        publish_offboard_control_mode(true, false); // 开位置，关速度
        publish_trajectory_setpoint(hover_x_, hover_y_, hover_z_, target_yaw_, false);

        // 🔥 核心防抖算法：必须连续 50 帧 (1秒) 速度和偏航角都极小，才算彻底平稳！
        if (speed < 0.2f && yaw_error < 0.1f) {
            stable_hold_counter_++;
        } else {
            stable_hold_counter_ = 0; // 只要晃动一下，立刻重新倒计时！
        }

        if (stable_hold_counter_ > 50) {
            current_state_ = SHOOTING;
            RCLCPP_INFO(node_->get_logger(), "🎯 [视觉伺服] 机身彻底平稳！[阶段3] 锁定目标，开始高清曝光拍摄...");
        }
    }
    // =========================================================
    // 🛡️ 阶段 3：高清拍摄倒计时 (Shooting)
    // =========================================================
    else if (current_state_ == SHOOTING) {
        publish_offboard_control_mode(true, false);
        publish_trajectory_setpoint(hover_x_, hover_y_, hover_z_, target_yaw_, false);

        hover_counter_++;
        if (hover_counter_ > 150) { // 3秒拍摄
            RCLCPP_INFO(node_->get_logger(), "✅ [HoverInspect] 拍摄完成！目标特征已保存。");
            
            // 🔥 核心修复 1B：使用锁死的真实坐标进行广播拉黑！
            geometry_msgs::msg::Point inspected_msg;
            inspected_msg.x = locked_defect_x_; 
            inspected_msg.y = locked_defect_y_; 
            inspected_msg.z = locked_defect_z_;
            inspected_pub_->publish(inspected_msg);
            
            config().blackboard->set<bool>("just_inspected", true);
            return BT::NodeStatus::SUCCESS;
        }
    }
    return BT::NodeStatus::RUNNING;
}

void HoverInspectAction::onHalted() {
    RCLCPP_WARN(node_->get_logger(), "[HoverInspect] ⚠️ 悬停拍摄被意外中断！");
}

void HoverInspectAction::publish_offboard_control_mode(bool position_control, bool velocity_control) {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = position_control;
    msg.velocity = velocity_control; // 动态切换模式！
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void HoverInspectAction::publish_trajectory_setpoint(float x, float y, float z, float yaw, bool is_velocity_mode) {
    px4_msgs::msg::TrajectorySetpoint msg{};
    // 🔥 PX4 规定：如果用速度模式，位置字段必须填 NaN！这是高级开发必知细节！
    if (is_velocity_mode) {
        msg.position = {NAN, NAN, NAN};
        msg.velocity = {x, y, z}; // 此时的 xyz 其实是传进来的速度(0,0,0)
    } else {
        msg.position = {x, y, z};
        msg.velocity = {NAN, NAN, NAN}; // 填 NaN 忽略速度
    }
    msg.yaw = yaw;
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

}
}