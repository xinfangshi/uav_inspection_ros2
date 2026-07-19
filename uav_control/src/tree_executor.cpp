#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point.hpp> // 引入视觉广播消息

// 引入所有行为树节点
#include "uav_control/behavior_trees/takeoff_action.hpp"
#include "uav_control/behavior_trees/goto_waypoint_action.hpp"
#include "uav_control/behavior_trees/check_defect_condition.hpp"
#include "uav_control/behavior_trees/hover_inspect_action.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("tree_executor_node");
    node->declare_parameter<std::string>("tree_xml_file", "uav_inspection_tree.xml");
    std::string xml_filename;
    node->get_parameter("tree_xml_file", xml_filename);

    // =========================================================
    // 🔥 架构之美：创建“黑板 (Blackboard)”，这是整个剧组的共享内存板
    // =========================================================
    auto blackboard = BT::Blackboard::create();
    blackboard->set<bool>("defect_detected", false); // 初始没发现

    // 建立顺风耳：偷听视觉大脑的广播，听到了就立刻写在黑板上！
    auto vision_sub = node->create_subscription<geometry_msgs::msg::Point>(
        "/vision/target_position", 10,
        [&blackboard](const geometry_msgs::msg::Point::SharedPtr msg) {
            if (msg->z > 0.5) { // z=1.0 表示发现目标
                blackboard->set<bool>("defect_detected", true);
                blackboard->set<double>("defect_u", msg->x);
                blackboard->set<double>("defect_v", msg->y);
            } else {
                blackboard->set<bool>("defect_detected", false);
            }
        }
    );

    BT::BehaviorTreeFactory factory;
    
    // 注册所有的演员！
    factory.registerNodeType<uav_control::behavior_trees::TakeoffAction>("Takeoff", node);
    factory.registerNodeType<uav_control::behavior_trees::GoToWaypointAction>("GoToWaypoint", node);
    factory.registerNodeType<uav_control::behavior_trees::HoverInspectAction>("HoverAndInspect", node);
    factory.registerNodeType<uav_control::behavior_trees::CheckDefectCondition>("CheckDefect");

    std::string package_path = ament_index_cpp::get_package_share_directory("uav_control");
    std::string xml_file = package_path + "/behavior_trees/" + xml_filename;

    RCLCPP_INFO(node->get_logger(), "🌳 正在加载带黑板的行为树: %s", xml_file.c_str());

    try {
        // 构建树，并把黑板发给树里的所有节点！
        auto tree = factory.createTreeFromFile(xml_file, blackboard);
        RCLCPP_INFO(node->get_logger(), "🚀 大闭环行为树开始执行！");
        
        rclcpp::Rate rate(50); 
        BT::NodeStatus status = BT::NodeStatus::RUNNING;

        while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
            rclcpp::spin_some(node);  // 处理 ROS 2 回调 (更新黑板)
            status = tree.tickOnce(); // 滴答一次树
            rate.sleep();             // 保持 50Hz
        }
        RCLCPP_INFO(node->get_logger(), "✅ 任务结束。");
    } 
    catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "❌ 行为树崩溃: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}