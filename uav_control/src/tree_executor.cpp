#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <array>
#include <cmath>

#include "uav_control/behavior_trees/takeoff_action.hpp"
#include "uav_control/behavior_trees/goto_waypoint_action.hpp"
#include "uav_control/behavior_trees/check_defect_condition.hpp"
#include "uav_control/behavior_trees/hover_inspect_action.hpp"
#include "uav_control/behavior_trees/land_action.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("tree_executor_node");
    node->declare_parameter<std::string>("tree_xml_file", "uav_inspection_tree.xml");
    std::string xml_filename;
    node->get_parameter("tree_xml_file", xml_filename);

    auto blackboard = BT::Blackboard::create();
    blackboard->set<bool>("defect_detected", false);
    blackboard->set<bool>("just_inspected", false);

    // 🔥 架构优化：黑名单逻辑下放给视觉节点，执行器只负责纯粹的监听与状态切换
    // =========================================================
    auto vision_sub = node->create_subscription<geometry_msgs::msg::Point>(
        "/vision/target_3d_position", 10,
        [&blackboard, node](const geometry_msgs::msg::Point::SharedPtr msg) {
            if (msg->z != -1.0) { 
                // 只要视觉报上来的，绝对是没拍过的合法新目标！直接触发警报！
                blackboard->set<bool>("defect_detected", true);
                blackboard->set<double>("defect_x", msg->x); 
                blackboard->set<double>("defect_y", msg->y); 
                blackboard->set<double>("defect_z", msg->z); 
            } else {
                blackboard->set<bool>("defect_detected", false);
            }
        }
    );

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<uav_control::behavior_trees::TakeoffAction>("Takeoff", node);
    factory.registerNodeType<uav_control::behavior_trees::GoToWaypointAction>("GoToWaypoint", node);
    factory.registerNodeType<uav_control::behavior_trees::HoverInspectAction>("HoverAndInspect", node);
    factory.registerNodeType<uav_control::behavior_trees::CheckDefectCondition>("CheckDefect");
    factory.registerNodeType<uav_control::behavior_trees::LandAction>("Land", node);

    std::string package_path = ament_index_cpp::get_package_share_directory("uav_control");
    std::string xml_file = package_path + "/behavior_trees/" + xml_filename;
    RCLCPP_INFO(node->get_logger(), "🌳 正在加载带黑板的行为树: %s", xml_file.c_str());

    try {
        auto tree = factory.createTreeFromFile(xml_file, blackboard);
        RCLCPP_INFO(node->get_logger(), "🚀 大闭环行为树开始执行！");
        
        rclcpp::Rate rate(50); 
        BT::NodeStatus status = BT::NodeStatus::RUNNING;

        while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
            rclcpp::spin_some(node);  
            
            // 拍完照后复位打断标志，允许继续巡航
            bool just_inspected = false;
            if (blackboard->get<bool>("just_inspected", just_inspected) && just_inspected) {
                blackboard->set<bool>("defect_detected", false);
                blackboard->set<bool>("just_inspected", false);
            }

            status = tree.tickOnce(); 
            rate.sleep();             
        }
        RCLCPP_INFO(node->get_logger(), "✅ 任务结束。");
    } 
    catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "❌ 行为树崩溃: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}