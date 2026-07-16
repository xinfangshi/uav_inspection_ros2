#include "uav_vision/detectors/dnn_detector.hpp" 
#include <iostream>
#include <cstdlib>

int main() 
{
    std::cout << "🚀 [独立 Demo] 启动真实的 OpenCV DNN 深度学习推理测试...\n";
    
    // 生成一张假图测试
    cv::Mat mock_image(400, 600, CV_8UC3, cv::Scalar(150, 150, 150));
    cv::line(mock_image, cv::Point(200, 150), cv::Point(250, 300), cv::Scalar(20, 20, 20), 5);

    // 获取系统的 HOME 目录，拼接模型绝对路径
    std::string home_dir = std::getenv("HOME");
    std::string txt_path = home_dir + "/ros2_ws/src/uav_inspection/uav_vision/models/mobilenet.prototxt";
    std::string bin_path = home_dir + "/ros2_ws/src/uav_inspection/uav_vision/models/mobilenet.caffemodel";

    // 💡 架构的魅力：依赖注入新大脑！
    uav_vision::DnnDetector ai_detector(txt_path, bin_path);
    
    std::cout << "🧠 真 AI 模型正在前向推理画面...\n";
    cv::Rect bbox = ai_detector.detect(mock_image);
    
    if (bbox.area() > 0) {
        std::cout << "🎯 发现物体！坐标: X=" << bbox.x << ", Y=" << bbox.y << "\n";
        cv::rectangle(mock_image, bbox, cv::Scalar(0, 0, 255), 2);
        cv::putText(mock_image, "AI Detected Object", cv::Point(bbox.x, bbox.y - 10), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
    } else {
        std::cout << "✅ 画面正常，AI 未检测到目标。\n";
    }

    cv::imshow("True AI Vision Demo", mock_image);
    cv::waitKey(0);
    
    return 0;
}