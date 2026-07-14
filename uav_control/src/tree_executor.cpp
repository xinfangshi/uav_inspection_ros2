#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "uav_control/behavior_trees/takeoff_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "uav_control/behavior_trees/goto_waypoint_action.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("tree_executor_node");

    node->declare_parameter<std::string>("tree_xml_file", "uav_inspection_tree.xml");
    std::string xml_filename;
    node->get_parameter("tree_xml_file", xml_filename);

    BT::BehaviorTreeFactory factory;

    // 🔥 架构核心：依赖注入！
    // 将 node 指针作为额外参数，传递给 TakeoffAction 的构造函数！
    factory.registerNodeType<uav_control::behavior_trees::TakeoffAction>("Takeoff", node);
    factory.registerNodeType<uav_control::behavior_trees::GoToWaypointAction>("GoToWaypoint", node);

    std::string package_path = ament_index_cpp::get_package_share_directory("uav_control");
    std::string xml_file = package_path + "/behavior_trees/" + xml_filename;

    RCLCPP_INFO(node->get_logger(), "🌳 正在加载行为树: %s", xml_file.c_str());

    try {
        auto tree = factory.createTreeFromFile(xml_file);
        RCLCPP_INFO(node->get_logger(), "🚀 行为树开始执行！");

        // 🔥 架构核心：自定义高频主循环 (50Hz)
        rclcpp::Rate rate(50);
        BT::NodeStatus status = BT::NodeStatus::RUNNING;

        // 只要树还在 RUNNING，并且 ROS 没被关闭，就持续以 50Hz 滴答它
        while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
            status = tree.tickOnce();  // 滴答一次
            rclcpp::spin_some(node);   // 处理一下 ROS 2 的回调
            rate.sleep();              // 睡 20ms，保持 50Hz
        }

        RCLCPP_INFO(node->get_logger(), "✅ 行为树执行完毕！最终状态: %s", BT::toStr(status).c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "❌ 行为树崩溃: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}