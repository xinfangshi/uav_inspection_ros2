#include "uav_vision/filters/kalman_filter.hpp"

namespace uav_vision {
namespace filters {

TargetKalmanFilter::TargetKalmanFilter() : is_initialized_(false) {
    // 状态量 x_ 和 协方差 P_ 会在 init() 中初始化

    // Q: 过程噪声 (我们假设裂缝是死物，所以模型极度可靠，过程噪声设得非常非常小)
    Q_ = Eigen::Matrix3d::Identity() * 0.001;

    // R: 观测噪声 (AI 画框的抖动 + 深度相机的红外噪点 + 飞机的悬停微震)
    // 🔥 核心修复 2：将观测噪声 R 从 0.5 暴力调大到 5.0 甚至 10.0！
    // 这意味着我们告诉 KF：“随着飞机移动，视觉算出的坐标极其不可靠（视角畸变），你给我死死守住它最初的平滑位置！”
    R_ = Eigen::Matrix3d::Identity() * 5.0;

    I_ = Eigen::Matrix3d::Identity();
}

void TargetKalmanFilter::init(double x, double y, double z) {
    x_ << x, y, z;
    // 刚开始时，我们对这个初始位置不是 100% 确定，给一个初始协方差
    P_ = Eigen::Matrix3d::Identity() * 1.0;
    is_initialized_ = true;
}

void TargetKalmanFilter::predict() {
    if (!is_initialized_) return;
    // 因为是静态目标，状态转移矩阵 F = I (x = F*x)
    // 预测协方差: P = F * P * F^T + Q => P = P + Q
    P_ = P_ + Q_;
}

void TargetKalmanFilter::update(double meas_x, double meas_y, double meas_z) {
    if (!is_initialized_) return;

    Eigen::Vector3d z_meas(meas_x, meas_y, meas_z);

    // 观测矩阵 H = I (我们直接观测 X, Y, Z)
    // 计算卡尔曼增益 K: K = P * H^T * (H * P * H^T + R)^-1 => K = P * (P + R)^-1
    Eigen::Matrix3d S = P_ + R_;
    Eigen::Matrix3d K = P_ * S.inverse();

    // 更新最优状态估计: x = x + K * (z_meas - H * x)
    Eigen::Vector3d y = z_meas - x_; // 测量残差 (Innovation)
    x_ = x_ + K * y;

    // 更新协方差: P = (I - K * H) * P => P = (I - K) * P
    P_ = (I_ - K) * P_;
}

Eigen::Vector3d TargetKalmanFilter::get_state() const {
    return x_;
}

bool TargetKalmanFilter::is_initialized() const {
    return is_initialized_;
}

void TargetKalmanFilter::reset() {
    is_initialized_ = false;
}

} // namespace filters
} // namespace uav_vision