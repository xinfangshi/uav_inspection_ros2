#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

namespace uav_vision {
namespace filters {

// 追踪目标结构体
struct TrackedTarget {
    int id;                 // 唯一的身份证号
    cv::Rect bbox;          // 当前的 2D 边框
    int time_since_update;  // 有多少帧没看到它了 (寿命)
    int hits;               // 连续看到了它多少帧 (信任度)
};

class LiteSORT {
public:
    // max_age: 目标丢失多少帧后注销; min_hits: 连续看多少帧才信任; iou_threshold: IOU匹配阈值
    LiteSORT(int max_age = 15, int min_hits = 1, float iou_threshold = 0.05);

    // 核心接口：输入一堆没名字的框，输出一堆带 ID 的追踪目标
    std::vector<TrackedTarget> update(const std::vector<cv::Rect>& detections);

private:
    int max_age_;
    int min_hits_;
    float iou_threshold_;
    int next_id_;
    std::vector<TrackedTarget> tracks_;

    // 计算两个矩形框的重合度 (Intersection Over Union)
    static float calculate_iou(const cv::Rect& a, const cv::Rect& b);
};

} // namespace filters
} // namespace uav_vision