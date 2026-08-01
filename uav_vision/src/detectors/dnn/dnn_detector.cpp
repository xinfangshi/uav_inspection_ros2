#include "uav_vision/detectors/dnn/dnn_detector.hpp"
#include <iostream>

namespace uav_vision
{

DnnDetector::DnnDetector(const std::string& model_txt, const std::string& model_bin)
{
    // 读取底层的深度学习模型
    net_ = cv::dnn::readNetFromCaffe(model_txt, model_bin);
    if (net_.empty()) {
        std::cerr << "❌ 致命错误：无法加载 DNN 模型！请检查路径。" << std::endl;
    } else {
        std::cout << "✅ OpenCV DNN 模型加载成功！" << std::endl;
    }
}

std::vector<cv::Rect> DnnDetector::detect(const cv::Mat& input_image)
{
    // 🔥 修复 1：如果图片为空，返回一个空的 vector 数组 (使用 {} 即可)
    if (input_image.empty()) return {};

    // 1. 预处理：将普通图像转换为神经网络认识的张量 (Blob)
    cv::Mat blob = cv::dnn::blobFromImage(input_image, 0.007843, cv::Size(300, 300), cv::Scalar(127.5, 127.5, 127.5), false, false);

    // 2. 塞入大脑：输入张量
    net_.setInput(blob);

    // 3. 神经放电：前向推理传播
    cv::Mat detections = net_.forward();

    // 4. 解析输出矩阵
    cv::Mat detectionMat(detections.size[2], detections.size[3], CV_32F, detections.ptr<float>());
    
    // 🔥 修复 2：定义一个数组，用来装所有的目标！删掉之前的 max_confidence
    std::vector<cv::Rect> bboxes;

    // 遍历 AI 找到的所有物体
    for (int i = 0; i < detectionMat.rows; i++) {
        float confidence = detectionMat.at<float>(i, 2); // 取出置信度
        
        // 🔥 修复 3：只要确信度 > 50% 的，全都要！不需要跟 max_confidence 比较了
        if (confidence > 0.5) {
            // 网络输出的坐标是 0~1 的比例，需要乘上原图的宽高
            int x1 = static_cast<int>(detectionMat.at<float>(i, 3) * input_image.cols);
            int y1 = static_cast<int>(detectionMat.at<float>(i, 4) * input_image.rows);
            int x2 = static_cast<int>(detectionMat.at<float>(i, 5) * input_image.cols);
            int y2 = static_cast<int>(detectionMat.at<float>(i, 6) * input_image.rows);
            
            // 把算好的目标框，塞进数组里
            bboxes.push_back(cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)));
        }
    }
    
    // 🔥 修复 4：完美返回装满目标的数组！
    return bboxes; 
}

} // namespace uav_vision