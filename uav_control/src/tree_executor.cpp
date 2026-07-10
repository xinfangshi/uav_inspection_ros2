#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "uav_control/behavior_trees/takeoff_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // 实例化一个 ROS 2 节点，专门用于管理执行器
    auto node = rclcpp::Node::make_shared("tree_executor_node");
    
    // 规范：声明一个名为 "tree_xml_file" 的参数，并设置默认值
    node->declare_parameter<std::string>("tree_xml_file", "uav_inspection_tree.xml");
    
    std::string xml_filename;
    node->get_parameter("tree_xml_file", xml_filename);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<uav_control::behavior_trees::TakeoffAction>("Takeoff");

    std::string package_path = ament_index_cpp::get_package_share_directory("uav_control");
    // 动态拼接路径，文件可以随时换！
    std::string xml_file = package_path + "/behavior_trees/" + xml_filename;

    RCLCPP_INFO(node->get_logger(), "🌳 正在加载行为树: %s", xml_file.c_str());

    try {
        auto tree = factory.createTreeFromFile(xml_file);
        RCLCPP_INFO(node->get_logger(), "🚀 行为树开始执行！");
        tree.tickWhileRunning();
        RCLCPP_INFO(node->get_logger(), "✅ 行为树执行完毕！任务结束。");
    } 
    catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "❌ 行为树崩溃: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}