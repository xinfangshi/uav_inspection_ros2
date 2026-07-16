#include <rclcpp/rclcpp.hpp>
#include "uav_vision/ros_nodes/camera_subscriber.hpp"
#include "uav_vision/detectors/dnn_detector.hpp" // 引入真脑
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // 1. 动态获取包的安装路径，告别绝对路径硬编码！
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("uav_vision");
    std::string txt_path = pkg_share_dir + "/models/mobilenet.prototxt";
    std::string bin_path = pkg_share_dir + "/models/mobilenet.caffemodel";

    std::cout << "🔄 正在加载深度学习模型..." << std::endl;

    // 2. 实例化真正的深度学习大脑！
    auto true_ai_brain = std::make_shared<uav_vision::DnnDetector>(txt_path, bin_path);
    
    // 3. 依赖注入：把 AI 大脑塞进眼睛节点！
    auto node = std::make_shared<uav_vision::CameraSubscriber>(true_ai_brain);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}