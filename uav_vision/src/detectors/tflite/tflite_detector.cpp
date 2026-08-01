#include "uav_vision/detectors/tflite/tflite_detector.hpp"
#include <iostream>

namespace uav_vision
{

TFLiteDetector::TFLiteDetector(const std::string& model_path)
{
    model_ = tflite::FlatBufferModel::BuildFromFile(model_path.c_str());
    if (!model_) {
        std::cerr << "❌ 致命错误：TFLite 模型加载失败！" << std::endl;
        return;
    }
    tflite::InterpreterBuilder builder(*model_, resolver_);
    builder(&interpreter_);
    interpreter_->AllocateTensors();
    std::cout << "✅ TensorFlow Lite (C++) 引擎加载成功，支持多目标并发识别！" << std::endl;
}

// 🔥 接口已升级为返回数组 std::vector<cv::Rect>
std::vector<cv::Rect> TFLiteDetector::detect(const cv::Mat& input_image)
{
    // 🔥 修复 1：所有的空返回，直接用 {} 表示返回空数组！
    if (input_image.empty() || !interpreter_) return {};

    cv::Mat resized_image;
    cv::resize(input_image, resized_image, cv::Size(300, 300));

    int input_idx = interpreter_->inputs()[0];
    TfLiteTensor* input_tensor = interpreter_->tensor(input_idx);

    if (!input_tensor) {
        std::cerr << "❌ 错误：无法获取模型的输入张量！" << std::endl;
        return {}; // 返回空数组
    }

    if (input_tensor->type == kTfLiteFloat32) {
        cv::Mat float_image;
        resized_image.convertTo(float_image, CV_32FC3, 2.0 / 255.0, -1.0);
        float* input_ptr = interpreter_->typed_input_tensor<float>(0);
        if (input_ptr) {
            memcpy(input_ptr, float_image.data, float_image.total() * float_image.elemSize());
        }
    } 
    else if (input_tensor->type == kTfLiteUInt8) {
        uint8_t* input_ptr = interpreter_->typed_input_tensor<uint8_t>(0);
        if (input_ptr) {
            memcpy(input_ptr, resized_image.data, resized_image.total() * resized_image.elemSize());
        }
    } 
    else {
        std::cerr << "❌ 错误：不支持的张量数据类型代码: " << input_tensor->type << std::endl;
        return {}; // 返回空数组
    }

    if (interpreter_->Invoke() != kTfLiteOk) {
        std::cerr << "❌ 错误：TFLite 推理执行失败！" << std::endl;
        return {}; // 返回空数组
    }

    // ========================================================
    // 🔥 修复 2：恢复 TFLite 专属的指针解析方式！
    // ========================================================
    float* boxes = interpreter_->typed_output_tensor<float>(0);
    float* scores = interpreter_->typed_output_tensor<float>(2);

    if (!boxes || !scores) {
        std::cerr << "⚠️ 提示：模型的输出张量不是 Float32 格式！" << std::endl;
        return {}; // 返回空数组
    }

    // 建立一个数组来装目标
    std::vector<cv::Rect> bboxes;

    // TFLite SSD 默认输出最多 10 个目标
    for (int i = 0; i < 10; ++i) {
        // 只要分数 > 50% 的，我全都要！
        if (scores[i] > 0.5) {
            // TFLite 输出的框是 [ymin, xmin, ymax, xmax] 比例
            int y1 = static_cast<int>(boxes[i * 4 + 0] * input_image.rows);
            int x1 = static_cast<int>(boxes[i * 4 + 1] * input_image.cols);
            int y2 = static_cast<int>(boxes[i * 4 + 2] * input_image.rows);
            int x2 = static_cast<int>(boxes[i * 4 + 3] * input_image.cols);
            
            // 把算好的框装进车厢
            bboxes.push_back(cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)));
        }
    }

    // 完美返回包含多目标的数组！
    return bboxes;
}

} // namespace uav_vision