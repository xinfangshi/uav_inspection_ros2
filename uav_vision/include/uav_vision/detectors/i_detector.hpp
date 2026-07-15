#pragma once

#include <opencv2/opencv.hpp>

namespace uav_vision
{

/**
 * @brief AI 缺陷检测器通用接口 (Interface)
 * 无论底层是 OpenCV, TFLite 还是 TensorRT，都必须实现这个接口
 */
class IDetector
{
public:
    // 虚析构函数，保证派生类能被正确释放
    virtual ~IDetector() = default;

    /**
     * @brief 核心推理接口
     * @param input_image 摄像头传入的 RGB 画面
     * @return cv::Rect 缺陷的 2D Bounding Box
     */
    virtual cv::Rect detect(const cv::Mat& input_image) = 0;
};

} // namespace uav_vision