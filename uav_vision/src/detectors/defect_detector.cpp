#include "uav_vision/detectors/defect_detector.hpp"

namespace uav_vision
{

MockDetector::MockDetector() {}

cv::Rect MockDetector::detect(const cv::Mat& input_image)
{
    // ====== 这里是 Mock AI 的逻辑 ======
    // 未来会把这段代码替换为: TFLite_Model.invoke(input_image)
    
    cv::Mat gray, thresh;
    // 1. 转为灰度图
    cv::cvtColor(input_image, gray, cv::COLOR_BGR2GRAY);
    // 2. 寻找极暗的区域（模拟黑色的裂缝），阈值设为 50
    cv::threshold(gray, thresh, 50, 255, cv::THRESH_BINARY_INV);
    
    std::vector<std::vector<cv::Point>> contours;
    // 3. 提取轮廓
    cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    if (!contours.empty()) {
        // 4. 找到面积最大的那条裂缝
        auto largest_contour = *std::max_element(contours.begin(), contours.end(),
            [](const auto& a, const auto& b) { return cv::contourArea(a) < cv::contourArea(b); });
            
        // 5. 返回它的 2D 边框！
        return cv::boundingRect(largest_contour);
    }
    
    // 如果没找到，返回一个面积为 0 的空框
    return cv::Rect(0, 0, 0, 0); 
}

} // namespace uav_vision