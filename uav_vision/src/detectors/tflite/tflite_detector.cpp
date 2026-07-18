#include "uav_vision/detectors/tflite/tflite_detector.hpp"
#include <iostream>

namespace uav_vision
{

TFLiteDetector::TFLiteDetector(const std::string& model_path)
{
    // 1. 加载 FlatBuffer 模型
    model_ = tflite::FlatBufferModel::BuildFromFile(model_path.c_str());
    if (!model_) {
        std::cerr << "❌ 致命错误：TFLite 模型加载失败！" << std::endl;
        return;
    }

    // 2. 构建解释器 (Interpreter)
    tflite::InterpreterBuilder builder(*model_, resolver_);
    builder(&interpreter_);
    if (!interpreter_) {
        std::cerr << "❌ 致命错误：TFLite 解释器构建失败！" << std::endl;
        return;
    }

    // 3. 分配内存张量 (Tensor Allocation)
    interpreter_->AllocateTensors();
    std::cout << "✅ TensorFlow Lite (C++) 模型加载并分配张量成功！" << std::endl;
}

cv::Rect TFLiteDetector::detect(const cv::Mat& input_image)
{
    if (input_image.empty() || !interpreter_) return cv::Rect(0, 0, 0, 0);

    // 1. 调整大小为模型标准尺寸 (300x300)
    cv::Mat resized_image;
    cv::resize(input_image, resized_image, cv::Size(300, 300));

    // ========================================================
    // 🔥 架构核心：动态检测张量类型，彻底消灭 NullPtr！
    // ========================================================
    int input_idx = interpreter_->inputs()[0];
    TfLiteTensor* input_tensor = interpreter_->tensor(input_idx);

    if (!input_tensor) {
        std::cerr << "❌ 错误：无法获取模型的输入张量！" << std::endl;
        return cv::Rect(0, 0, 0, 0);
    }

    // 2. 根据模型类型，自动转换并填入数据
    if (input_tensor->type == kTfLiteFloat32) {
        // 模型要求 32 位浮点数 (通常需要归一化到 [-1, 1])
        cv::Mat float_image;
        resized_image.convertTo(float_image, CV_32FC3, 2.0 / 255.0, -1.0);
        float* input_ptr = interpreter_->typed_input_tensor<float>(0);
        if (input_ptr) {
            memcpy(input_ptr, float_image.data, float_image.total() * float_image.elemSize());
        }
    } 
    else if (input_tensor->type == kTfLiteUInt8) {
        // 模型要求 8 位无符号整数 (量化模型，直接传 0-255 的像素值即可)
        uint8_t* input_ptr = interpreter_->typed_input_tensor<uint8_t>(0);
        if (input_ptr) {
            memcpy(input_ptr, resized_image.data, resized_image.total() * resized_image.elemSize());
        }
    } 
    else {
        std::cerr << "❌ 错误：不支持的张量数据类型代码: " << input_tensor->type << std::endl;
        return cv::Rect(0, 0, 0, 0);
    }

    // 3. 执行前向推理
    if (interpreter_->Invoke() != kTfLiteOk) {
        std::cerr << "❌ 错误：TFLite 推理执行失败！" << std::endl;
        return cv::Rect(0, 0, 0, 0);
    }

    // 4. 解析输出结果 (大多数 SSD 模型的输出都是 Float32)
    float* boxes = interpreter_->typed_output_tensor<float>(0);
    float* scores = interpreter_->typed_output_tensor<float>(2);

    if (!boxes || !scores) {
        std::cerr << "⚠️ 提示：模型的输出张量不是 Float32 格式，请检查模型结构！" << std::endl;
        return cv::Rect(0, 0, 0, 0);
    }

    float max_confidence = 0.0;
    cv::Rect best_bbox(0, 0, 0, 0);

    for (int i = 0; i < 10; ++i) {
        if (scores[i] > 0.5 && scores[i] > max_confidence) {
            max_confidence = scores[i];
            int y1 = static_cast<int>(boxes[i * 4 + 0] * input_image.rows);
            int x1 = static_cast<int>(boxes[i * 4 + 1] * input_image.cols);
            int y2 = static_cast<int>(boxes[i * 4 + 2] * input_image.rows);
            int x2 = static_cast<int>(boxes[i * 4 + 3] * input_image.cols);
            best_bbox = cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2));
        }
    }
    return best_bbox;
}

}