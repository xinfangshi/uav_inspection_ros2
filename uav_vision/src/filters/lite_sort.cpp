#include "uav_vision/filters/lite_sort.hpp"
#include <algorithm>

namespace uav_vision {
namespace filters {

LiteSORT::LiteSORT(int max_age, int min_hits, float iou_threshold)
    : max_age_(max_age), min_hits_(min_hits), iou_threshold_(iou_threshold), next_id_(1) {}

float LiteSORT::calculate_iou(const cv::Rect& a, const cv::Rect& b) {
    cv::Rect intersection = a & b;
    float inter_area = intersection.area();
    float union_area = a.area() + b.area() - inter_area;
    if (union_area < 1e-5) return 0.0f;
    return inter_area / union_area;
}

std::vector<TrackedTarget> LiteSORT::update(const std::vector<cv::Rect>& detections) {
    // 1. 如果没有检测到任何东西，所有老目标寿命衰减
    if (detections.empty()) {
        for (auto& trk : tracks_) trk.time_since_update++;
        // 剔除过期的目标
        tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                      [this](const TrackedTarget& t) { return t.time_since_update > max_age_; }),
                      tracks_.end());
        return tracks_;
    }

    std::vector<bool> matched_detections(detections.size(), false);
    std::vector<bool> matched_tracks(tracks_.size(), false);

    // 2. 贪心 IOU 匹配 (大厂轻量化关联算法)
    for (size_t d = 0; d < detections.size(); ++d) {
        int best_trk_idx = -1;
        float best_iou = iou_threshold_;

        for (size_t t = 0; t < tracks_.size(); ++t) {
            if (matched_tracks[t]) continue; // 已经被别人认领了
            
            float iou = calculate_iou(detections[d], tracks_[t].bbox);
            if (iou > best_iou) {
                best_iou = iou;
                best_trk_idx = t;
            }
        }

        // 关联成功！更新老目标
        if (best_trk_idx >= 0) {
            tracks_[best_trk_idx].bbox = detections[d];
            tracks_[best_trk_idx].time_since_update = 0; // 续命
            tracks_[best_trk_idx].hits++;                // 增加信任度
            matched_detections[d] = true;
            matched_tracks[best_trk_idx] = true;
        }
    }

    // 3. 处理没匹配上的老目标 (衰减寿命)
    for (size_t t = 0; t < tracks_.size(); ++t) {
        if (!matched_tracks[t]) {
            tracks_[t].time_since_update++;
        }
    }

    // 4. 剔除过期目标
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                  [this](const TrackedTarget& t) { return t.time_since_update > max_age_; }),
                  tracks_.end());

    // 5. 处理没匹配上的新检测框 (诞生新目标，分配新 ID)
    for (size_t d = 0; d < detections.size(); ++d) {
        if (!matched_detections[d]) {
            TrackedTarget new_trk;
            new_trk.id = next_id_++;
            new_trk.bbox = detections[d];
            new_trk.time_since_update = 0;
            new_trk.hits = 1;
            tracks_.push_back(new_trk);
        }
    }

    // 6. 过滤输出：只返回那些连续看到好几帧的“靠谱目标”！(剔除闪烁噪点)
    std::vector<TrackedTarget> active_tracks;
    for (const auto& trk : tracks_) {
        if (trk.time_since_update < 1 && trk.hits >= min_hits_) {
            active_tracks.push_back(trk);
        }
    }

    return active_tracks;
}

}
}