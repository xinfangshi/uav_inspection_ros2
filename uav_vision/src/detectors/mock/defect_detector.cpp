#include "uav_vision/detectors/mock/defect_detector.hpp"

namespace uav_vision
{

MockDetector::MockDetector() {}

// 🔥 接口升级：返回值改为 std::vector<cv::Rect>
std::vector<cv::Rect> MockDetector::detect(const cv::Mat& input_image)
{
    cv::Mat gray, thresh;
    cv::cvtColor(input_image, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, thresh, 120, 255, cv::THRESH_BINARY_INV);
    
    // =========================================================================
    // 🛡️ 工业级视觉优化 1：ROI 盲区遮罩 (Self-Occlusion Masking)
    // 强制把画面边缘（可能出现机翼、螺旋桨的地方）涂黑，彻底屏蔽！
    // =========================================================================
    int w = thresh.cols;
    int h = thresh.rows;
    
    // 屏蔽画面最上方 25% (通常是上方两个螺旋桨)
    cv::rectangle(thresh, cv::Point(0, 0), cv::Point(w, h * 0.25), cv::Scalar(0), cv::FILLED);
    // 屏蔽画面左右各 15% 的边缘
    cv::rectangle(thresh, cv::Point(0, 0), cv::Point(w * 0.15, h), cv::Scalar(0), cv::FILLED);
    cv::rectangle(thresh, cv::Point(w * 0.85, 0), cv::Point(w, h), cv::Scalar(0), cv::FILLED);
    // 屏蔽画面最下方 10% (机腹起落架可能露出的地方)
    cv::rectangle(thresh, cv::Point(0, h * 0.9), cv::Point(w, h), cv::Scalar(0), cv::FILLED);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // 🔥 升级点：创建一个数组，存放所有找到的目标！
    std::vector<cv::Rect> bboxes;

    // =========================================================================
    // 🛡️ 工业级视觉优化 2：形态学面积滤波 (Area Filtering)
    // 防止画面里有个很小的黑点(比如远处的噪点)引发打断。
    // =========================================================================
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        
        // 只识别面积大于 500 个像素的有效方块/裂缝，并且排除可能占据整个屏幕的异常巨大黑影
        // 🔥 注意：去掉了 `area > max_area` 的争第一逻辑，现在只要及格，全部招募！
        if (area > 500 && area < (w * h * 0.5)) {
            bboxes.push_back(cv::boundingRect(contour));
        }
    }
    
    // 返回包含所有目标的数组（如果画面干净，返回的就是一个空数组 .empty() ）
    return bboxes; 
}

} // namespace uav_vision