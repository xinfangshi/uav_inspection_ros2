#include <rclcpp/rclcpp.hpp>
#include "uav_vision/ros_nodes/camera_subscriber.hpp"
// 只有在这个最高层的执行器里，才引入具体的算法实现
#include "uav_vision/detectors/defect_detector.hpp" 

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // 1. 实例化一个 Mock 假 AI 大脑
    auto mock_ai = std::make_shared<uav_vision::MockDetector>();
    
    // 2. 把大脑注入进 ROS 2 眼睛节点里！
    auto node = std::make_shared<uav_vision::CameraSubscriber>(mock_ai);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}