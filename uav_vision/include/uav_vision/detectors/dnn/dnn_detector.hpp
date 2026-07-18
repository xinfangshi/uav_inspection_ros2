#pragma once

#include "uav_vision/detectors/i_detector.hpp"
#include <opencv2/dnn.hpp>
#include <string>

namespace uav_vision
{

class DnnDetector : public IDetector
{
public:
    // 构造函数：需要传入刚才下载的模型文件路径
    DnnDetector(const std::string& model_txt, const std::string& model_bin);

    // 真正的深度学习前向推理！
    cv::Rect detect(const cv::Mat& input_image) override;

private:
    cv::dnn::Net net_; // OpenCV DNN 核心网络对象
};

} // namespace uav_vision