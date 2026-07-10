#pragma once

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/action_node.h"

// 严格遵守命名空间规范，防止在大项目中发生命名冲突
namespace uav_control
{
namespace behavior_trees
{

/**
 * @brief 负责执行无人机起飞动作的行为树节点
 * 继承自 BT::SyncActionNode (同步动作节点)
 */
class TakeoffAction : public BT::SyncActionNode
{
public:
    // 构造函数：接收 XML 中的节点名称和配置
    TakeoffAction(const std::string& name, const BT::NodeConfig& config);

    // 必须实现的静态方法：告诉行为树这个节点需要接收 XML 里的什么参数
    static BT::PortsList providedPorts();

    // 必须实现的核心重载方法：行为树运行到这里时，具体执行什么逻辑
    BT::NodeStatus tick() override;

private:
    // 未来我们会在这里存放与 ROS 2 通信的 Publisher 指针
    // rclcpp::Node::SharedPtr ros_node_; 
};

} // namespace behavior_trees
} // namespace uav_control