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

    // =========================================================
    // 🔥 架构之美：引入 3D 空间记忆数据库 (黑名单)
    // =========================================================
    std::vector<std::array<double, 3>> inspected_targets;

    auto vision_sub = node->create_subscription<geometry_msgs::msg::Point>(
        "/vision/target_3d_position", 10,
        [&blackboard, &inspected_targets](const geometry_msgs::msg::Point::SharedPtr msg) {
            if (msg->z != -1.0) { 
                // 收到目标！先进行记忆库查重！
                bool is_new_target = true;
                for (const auto& target : inspected_targets) {
                    // 计算 3D 空间欧式距离
                    double dist = std::sqrt(std::pow(msg->x - target[0], 2) + 
                                            std::pow(msg->y - target[1], 2) + 
                                            std::pow(msg->z - target[2], 2));
                    // 如果该坐标在已巡检目标的 2.0 米范围内，判定为重复目标！
                    if (dist < 2.0) {
                        is_new_target = false;
                        break;
                    }
                }

                if (is_new_target) {
                    blackboard->set<bool>("defect_detected", true);
                    blackboard->set<double>("defect_x", msg->x); 
                    blackboard->set<double>("defect_y", msg->y); 
                    blackboard->set<double>("defect_z", msg->z); 
                } else {
                    blackboard->set<bool>("defect_detected", false); // 老目标，果断无视！
                }
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
            
            // =========================================================
            // 🔥 记忆更新系统：如果刚拍完照，立刻将其坐标录入黑名单！
            // =========================================================
            bool just_inspected = false;
            if (blackboard->get<bool>("just_inspected", just_inspected) && just_inspected) {
                double x = 0, y = 0, z = 0;
                
                // 🔥 架构师修复：严格判断返回值，彻底消除 nodiscard 警告！
                if (blackboard->get<double>("defect_x", x) &&
                    blackboard->get<double>("defect_y", y) &&
                    blackboard->get<double>("defect_z", z)) 
                {
                    inspected_targets.push_back({x, y, z}); // 加入黑名单
                    blackboard->set<bool>("just_inspected", false); // 状态复位
                    // ========================================================
                    // 🔥 核心修复：强行擦除警报标志位！
                    // 等待视觉节点的下一帧去重新评估黑名单，防止异步回调时间差导致的重复触发！
                    // ========================================================
                    blackboard->set<bool>("defect_detected", false); 
                    
                    RCLCPP_INFO(node->get_logger(), "🧠 [空间记忆] 已将目标 (X:%.2f, Y:%.2f, Z:%.2f) 录入已巡检数据库，两米内目标将被忽略！", x, y, z);
                } else {
                    RCLCPP_WARN(node->get_logger(), "⚠️ [空间记忆] 无法从黑板完整读取 3D 坐标！");
                    blackboard->set<bool>("just_inspected", false);
                }
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