#pragma once

#include "uav_vision/detectors/i_detector.hpp" // 引入接口

namespace uav_vision
{

// 继承 IDetector，实现具体的 OpenCV 模拟检测
class MockDetector : public IDetector
{
public:
    MockDetector();

    //重写父类方法
    cv::Rect detect(const cv::Mat& input_image) override;
};

} // namespace uav_vision