#include <rclcpp/rclcpp.hpp>
#include "uav_vision/ros_nodes/camera_subscriber.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <string>
#include <vector>

// 🔥 架构之美：引入所有的 AI 策略头文件
#include "uav_vision/detectors/mock/defect_detector.hpp"
#include "uav_vision/detectors/dnn/dnn_detector.hpp"
#include "uav_vision/detectors/tflite/tflite_detector.hpp"

int main(int argc, char * argv[])
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    
    // 1. 获取安全的命令行参数 (剔除掉 ROS 2 自带的内部参数)
    std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
    
    // 默认使用 dnn 引擎
    std::string engine = "dnn"; 
    if (args.size() > 1) {
        engine = args[1]; // 如果用户传了参数，就覆盖默认值
    }

    std::cout << "🚀 [视觉节点] 启动！当前指定 AI 引擎: [" << engine << "]" << std::endl;

    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("uav_vision");
    std::shared_ptr<uav_vision::IDetector> active_brain;

    // 2. 根据参数，多态实例化对应的 AI 大脑
    if (engine == "mock") {
        std::cout << "🔄 正在加载 [OpenCV 传统算子] 模拟引擎..." << std::endl;
        active_brain = std::make_shared<uav_vision::MockDetector>();
    } 
    else if (engine == "tflite") {
        std::cout << "🔄 正在加载 [TensorFlow Lite C++] 深度学习引擎..." << std::endl;
        std::string tflite_model = pkg_share_dir + "/models/model.tflite";
        active_brain = std::make_shared<uav_vision::TFLiteDetector>(tflite_model);
    } 
    else {
        std::cout << "🔄 正在加载 [OpenCV DNN] 深度学习引擎..." << std::endl;
        std::string txt_path = pkg_share_dir + "/models/mobilenet.prototxt";
        std::string bin_path = pkg_share_dir + "/models/mobilenet.caffemodel";
        active_brain = std::make_shared<uav_vision::DnnDetector>(txt_path, bin_path);
    }

    // 3. 依赖注入：把激活的 AI 大脑塞进眼睛节点！
    auto node = std::make_shared<uav_vision::CameraSubscriber>(active_brain);
    
    // 4. 让节点开始死循环监听 Gazebo 的图像
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}