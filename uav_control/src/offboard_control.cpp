#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <stdint.h>
#include <chrono>

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("offboard_control"), setpoint_counter_(0) {
        // 创建发布者：用于发布控制模式、轨迹点和车辆指令
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        // 创建一个 100ms (10Hz) 的定时器，这是 PX4 要求的最低高频心跳
        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "🚀 Offboard 控制节点已启动，准备起飞...");
    }

private:
    void timer_callback() {
        if (setpoint_counter_ == 50) {
            // 在发送了10个位置点(1秒)之后，切换到 Offboard 模式
            this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            // 解锁无人机 (Arm)
            this->arm();
        }

        // 发送 Offboard 控制模式心跳
        publish_offboard_control_mode();
        // 发送目标位置 (悬停在 5 米高空)
        publish_trajectory_setpoint();

        if (setpoint_counter_ < 51) {
            setpoint_counter_++;
        }
    }

    void arm() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(this->get_logger(), "⚔️ 发送解锁(Arm)指令!");
    }

    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_trajectory_setpoint() {
        px4_msgs::msg::TrajectorySetpoint msg{};
        // 注意：PX4 使用 NED (北-东-地) 坐标系！
        // Z轴向下为正，向上为负。所以 -5.0 代表飞向 5 米高空！
        msg.position = {0.0, 0.0, -5.0};
        // 航向角设为0
        msg.yaw = 0.0; 
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    uint64_t setpoint_counter_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}