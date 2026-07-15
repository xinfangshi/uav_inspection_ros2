#include "uav_vision/detectors/defect_detector.hpp"
#include <iostream>

int main() 
{
    std::cout << "🚀 [独立 Demo] 启动边缘 AI 视觉缺陷检测器测试...\n";
    
    // 1. 我们在内存里凭空“画”一张模拟的桥梁墙面图片 (灰色背景)
    cv::Mat mock_image(400, 600, CV_8UC3, cv::Scalar(150, 150, 150));
    // 在图片中间，用代码画一条黑色的“裂缝”
    cv::line(mock_image, cv::Point(200, 150), cv::Point(250, 300), cv::Scalar(20, 20, 20), 5);
    cv::line(mock_image, cv::Point(250, 300), cv::Point(300, 350), cv::Scalar(20, 20, 20), 4);

    // 2. 实例化我们的 AI 检测器大脑
    uav_vision::MockDetector ai_detector;
    
    // 3. 运行推理核心接口！
    std::cout << "🧠 AI 正在分析画面...\n";
    cv::Rect bbox = ai_detector.detect(mock_image);
    
    // 4. 解析输出结果，并把识别框画在图上
    if (bbox.area() > 0) {
        std::cout << "🎯 发现缺陷！坐标: X=" << bbox.x << ", Y=" << bbox.y 
                  << ", 宽=" << bbox.width << ", 高=" << bbox.height << "\n";
        // 画一个红色的粗识别框 (大厂 AI 标配视觉)
        cv::rectangle(mock_image, bbox, cv::Scalar(0, 0, 255), 2);
        // 写上标签文字
        cv::putText(mock_image, "Defect (AI Mock)", cv::Point(bbox.x, bbox.y - 10), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
    } else {
        std::cout << "✅ 画面正常，未发现缺陷。\n";
    }

    // 5. 弹窗展示大牛成果！
    cv::imshow("AI Vision Demo", mock_image);
    std::cout << "💡 提示: 在弹出的图片窗口上按键盘任意键退出测试...\n";
    cv::waitKey(0);
    
    return 0;
}