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

// 1. 收到 PX4 数据时，立刻翻译为 ENU！
void HoverInspectAction::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    current_x_ = msg->position[1];  // ENU X
    current_y_ = msg->position[0];  // ENU Y
    current_z_ = -msg->position[2]; // ENU Z
    current_vx_ = msg->velocity[1];
    current_vy_ = msg->velocity[0];
    
    // =========================================================
    // 🔥 核心修复：先用一个普通的 double 变量算好角度，避开 atomic 运算符限制！
    // =========================================================
    double ned_yaw = get_yaw_from_quaternion(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    double enu_yaw = M_PI / 2.0 - ned_yaw; 
    
    // 在局部变量里做 while 加减法，非常安全！
    while (enu_yaw > M_PI) enu_yaw -= 2.0 * M_PI;
    while (enu_yaw < -M_PI) enu_yaw += 2.0 * M_PI;
    
    // 算好归一化之后，一次性原子赋值给 current_yaw_！
    current_yaw_ = static_cast<float>(enu_yaw); 
    // // 1. 打印从 PX4 读到的原生 NED Yaw，以及我们转换后的 ENU Yaw
    // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, 
    // "🔍 [Odom 探测] PX4 原生 NED Yaw: %.2f | 转换后 ROS ENU Yaw: %.2f", 
    // ned_yaw, current_yaw_.load());
    
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
    // //
    // RCLCPP_INFO(node_->get_logger(), 
    // "🎯 [锁定探测] 目标坐标: (X:%.2f, Y:%.2f), 飞机坐标: (X:%.2f, Y:%.2f)", 
    // locked_defect_x_, locked_defect_y_, 
    // current_x_.load(), current_y_.load());
    // RCLCPP_INFO(node_->get_logger(), 
    // "🎯 [锁定探测] 计算得出目标 ENU Yaw: %.2f, 刹车锁死 ENU Yaw: %.2f", 
    // target_yaw_, brake_yaw_);

    current_state_ = BRAKING;
    stable_hold_counter_ = 0; hover_counter_ = 0;
    RCLCPP_INFO(node_->get_logger(), "📸 [视觉伺服] 触发！锁死目标坐标 (X:%.2f, Y:%.2f)，开始平滑刹车...", locked_defect_x_, locked_defect_y_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoverInspectAction::onRunning() {
    float speed = std::sqrt(current_vx_ * current_vx_ + current_vy_ * current_vy_);
    float yaw_error = std::abs(target_yaw_ - current_yaw_);
    if (yaw_error > M_PI) yaw_error = 2 * M_PI - yaw_error;
    // //
    // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, 
    // "⚖️ [误差探测] 当前状态: %d | 飞机 ENU Yaw: %.2f | 目标 ENU Yaw: %.2f | Yaw误差: %.2f", 
    // current_state_, current_yaw_.load(), target_yaw_, yaw_error);

    // =========================================================
    // 🛡️ 阶段 1：柔性刹车 (Velocity Control)
    // =========================================================
    if (current_state_ == BRAKING) {
        publish_offboard_control_mode(false, true); // 关位置，开速度控制
        // 🔥 核心修复 2B：刹车时绝对不许转头！死死锁住 brake_yaw_！
        publish_trajectory_setpoint(0.0, 0.0, 0.0, brake_yaw_, true); 
        // 当物理速度降到 0.5m/s 以下，说明刹车基本完成，切入阶段 2
        if (speed < 0.5f) {
            current_state_ = ALIGNING;
            RCLCPP_INFO(node_->get_logger(), "📸 [视觉伺服] 刹车完毕！[阶段2] 开始纯速度伺服，柔性旋转对焦...");
        }
    }
    // =========================================================
    // 🛡️ 阶段 2：姿态稳定校验 (Pure Velocity Control + Yaw Alignment)
    // =========================================================
    else if (current_state_ == ALIGNING) {
        // 🔥 终极修复 3：彻底抛弃位置控制！全程使用速度 0 悬停，防止钟摆效应！
        publish_offboard_control_mode(false, true); // 关位置，开速度
        publish_trajectory_setpoint(0.0, 0.0, 0.0, target_yaw_, true); // 速度为0，缓慢转机头

        // 🔥 核心防抖算法：必须连续 50 帧 (1秒) 速度和偏航角都极小，才算彻底平稳！
        if (speed < 0.2f && yaw_error < 0.15f) {
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
        // 🔥 终极修复 4：拍摄期间也死死保持速度 0 控制，稳如泰山！
        publish_offboard_control_mode(false, true);
        publish_trajectory_setpoint(0.0, 0.0, 0.0, target_yaw_, true);

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

// 2. 发给 PX4 之前，翻译回 NED！
void HoverInspectAction::publish_trajectory_setpoint(float x, float y, float z, float yaw, bool is_velocity_mode) {
    px4_msgs::msg::TrajectorySetpoint msg{};
    
    // =========================================================
    // 🔥 终极补丁 A：填补 ROS2 默认值陷阱！极其关键！
    // 必须显式将不用到的控制量设为 NAN，否则它们默认为 0.0。
    // PX4 收到 yawspeed=0.0 会与 target_yaw 发生控制器死锁，导致翻车！
    // =========================================================
    msg.acceleration = {NAN, NAN, NAN};
    msg.jerk = {NAN, NAN, NAN};
    msg.yawspeed = NAN; // 放开角速度限制，让 PX4 姿态环自行平滑转头！

    if (is_velocity_mode) {
        msg.position = {NAN, NAN, NAN};
        msg.velocity = {y, x, -z}; // ENU to NED
    } else {
        msg.position = {y, x, -z}; // ENU to NED
        msg.velocity = {NAN, NAN, NAN};
    }
    
    // =========================================================
    // 🔥 终极修复 5：严格归一化目标偏航角！彻底终结 PX4 发疯旋转！
    // =========================================================
    float ned_target_yaw = M_PI / 2.0 - yaw;
    while (ned_target_yaw > M_PI) ned_target_yaw -= 2.0 * M_PI;
    while (ned_target_yaw < -M_PI) ned_target_yaw += 2.0 * M_PI;
    
    msg.yaw = ned_target_yaw;    // ENU Yaw 转 NED Yaw (已归一化)
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    // //
    // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, 
    // "📤 [发送探测] 指令 ENU Yaw: %.2f => 逆向转换回 NED Yaw: %.2f", 
    // yaw, ned_target_yaw);

    trajectory_setpoint_publisher_->publish(msg);
}

}
}