#include "uav_control/behavior_trees/takeoff_action.hpp"

namespace uav_control
{
namespace behavior_trees
{

TakeoffAction::TakeoffAction(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
{
    // 这里可以进行初始化，比如从 config 中提取 ROS 2 的 Node 指针 (后续我们再深度集成)
}

BT::PortsList TakeoffAction::providedPorts()
{
    // 对应我们 XML 里写的: <Takeoff target_altitude="5.0"/>
    // 定义一个输入端口 "target_altitude"，类型为 double
    return { BT::InputPort<double>("target_altitude") };
}

BT::NodeStatus TakeoffAction::tick()
{
    double altitude = 0.0;
    
    // 从 XML 或黑板中读取 target_altitude 的值
    if (!getInput("target_altitude", altitude))
    {
        // 如果读取失败（比如 XML 里没写这个属性），抛出异常
        throw BT::RuntimeError("TakeoffAction 节点缺少 'target_altitude' 参数!");
    }

    // --- 这里是我们后续要接入的具体 ROS 2 PX4 起飞代码 ---
    // RCLCPP_INFO(rclcpp::get_logger("TakeoffAction"), "接收到起飞指令，目标高度: %.2f 米", altitude);
    
    // 这里简单用 std::cout 模拟打印 (因为还没集成 rclcpp logger)
    std::cout << "[BT NODE: Takeoff] 🚀 正在执行起飞，目标高度: " << altitude << " 米" << std::endl;

    // 返回 SUCCESS，告诉行为树：起飞动作完成，你可以去执行下一个节点了！
    return BT::NodeStatus::SUCCESS;
}

} // namespace behavior_trees
} // namespace uav_control