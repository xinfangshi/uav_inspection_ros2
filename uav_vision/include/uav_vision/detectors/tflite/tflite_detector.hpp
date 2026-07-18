#pragma once

#include "uav_vision/detectors/i_detector.hpp"
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include <string>
#include <memory>

namespace uav_vision
{

// 继承统一的 IDetector 接口
class TFLiteDetector : public IDetector
{
public:
    TFLiteDetector(const std::string& model_path);
    ~TFLiteDetector() override = default;

    cv::Rect detect(const cv::Mat& input_image) override;

private:
    // TFLite 核心组件
    std::unique_ptr<tflite::FlatBufferModel> model_;
    tflite::ops::builtin::BuiltinOpResolver resolver_;
    std::unique_ptr<tflite::Interpreter> interpreter_;
};

} // namespace uav_vision