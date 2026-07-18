#include "uav_vision/detectors/mock/defect_detector.hpp"
#include "uav_vision/detectors/dnn/dnn_detector.hpp"
#include "uav_vision/detectors/tflite/tflite_detector.hpp"
#include "uav_vision/detectors/i_detector.hpp"

#include <iostream>
#include <cstdlib>
#include <string>
#include <memory>

int main(int argc, char * argv[]) 
{
    // 1. 命令行参数解析：默认使用 dnn，如果有输入参数则使用指定引擎
    std::string engine = "dnn";
    if (argc > 1) {
        engine = argv[1];
    }

    std::cout << "🚀 [独立 Demo] 启动统一视觉测试工具，当前引擎: [" << engine << "]\n";

    // 2. 生成假图测试
    cv::Mat mock_image(400, 600, CV_8UC3, cv::Scalar(150, 150, 150));
    cv::line(mock_image, cv::Point(200, 150), cv::Point(250, 300), cv::Scalar(20, 20, 20), 5);

    std::string home_dir = std::getenv("HOME");
    
    // 3. 🔥 架构的终极魅力：定义一个通用的接口指针！
    std::shared_ptr<uav_vision::IDetector> active_brain;

    // 4. 根据输入参数，多态实例化具体的大脑
    if (engine == "mock") {
        std::cout << "🔄 正在加载 [OpenCV 传统算子] 模拟引擎...\n";
        active_brain = std::make_shared<uav_vision::MockDetector>();
    } 
    else if (engine == "tflite") {
        std::cout << "🔄 正在加载 [TensorFlow Lite C++] 深度学习引擎...\n";
        std::string tflite_path = home_dir + "/ros2_ws/src/uav_inspection/uav_vision/models/model.tflite";
        active_brain = std::make_shared<uav_vision::TFLiteDetector>(tflite_path);
    } 
    else {
        // 默认 dnn
        std::cout << "🔄 正在加载 [OpenCV DNN] 深度学习引擎...\n";
        std::string txt_path = home_dir + "/ros2_ws/src/uav_inspection/uav_vision/models/mobilenet.prototxt";
        std::string bin_path = home_dir + "/ros2_ws/src/uav_inspection/uav_vision/models/mobilenet.caffemodel";
        active_brain = std::make_shared<uav_vision::DnnDetector>(txt_path, bin_path);
    }

    // 5. 不管是什么大脑，推理接口只有唯一的一个：detect！
    std::cout << "🧠 AI 正在分析画面...\n";
    cv::Rect bbox = active_brain->detect(mock_image);
    
    // 6. 绘制结果
    if (bbox.area() > 0) {
        std::cout << "🎯 发现目标！坐标: X=" << bbox.x << ", Y=" << bbox.y << "\n";
        cv::rectangle(mock_image, bbox, cv::Scalar(0, 0, 255), 2);
        cv::putText(mock_image, "AI Detected (" + engine + ")", cv::Point(bbox.x, bbox.y - 10), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
    } else {
        std::cout << "✅ 画面正常，AI 未检测到目标。\n";
    }

    cv::imshow("Unified Vision Demo", mock_image);
    std::cout << "💡 提示: 按任意键退出...\n";
    cv::waitKey(0);
    
    return 0;
}