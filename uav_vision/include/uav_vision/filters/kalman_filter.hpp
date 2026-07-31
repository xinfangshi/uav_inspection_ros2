#pragma once

#include <Eigen/Dense>

namespace uav_vision {
namespace filters {

/**
 * @brief 针对 3D 静态目标的线性卡尔曼滤波器 (Linear Kalman Filter)
 * 用于平滑 AI 视觉 + 深度相机 + TF2 算出的带有高频噪声的绝对坐标
 */
class TargetKalmanFilter {
public:
    TargetKalmanFilter();

    // 初始化滤波器 (传入第一次观测到的坐标)
    void init(double x, double y, double z);

    // 状态预测 (由于是静态目标，预测它在原地不动)
    void predict();

    // 观测更新 (融合最新的、带噪声的观测坐标)
    void update(double meas_x, double meas_y, double meas_z);

    // 获取当前被滤波器“洗干净”的最佳 3D 坐标
    Eigen::Vector3d get_state() const;

    // 判断滤波器是否已经初始化
    bool is_initialized() const;

    // 重置滤波器 (用于无人机去寻找下一个新目标时)
    void reset();

private:
    bool is_initialized_;

    Eigen::Vector3d x_; // 状态向量 [X, Y, Z]^T
    Eigen::Matrix3d P_; // 状态协方差矩阵 (表示我们对当前状态的信任度)
    Eigen::Matrix3d Q_; // 过程噪声协方差 (模型自身的误差)
    Eigen::Matrix3d R_; // 观测噪声协方差 (传感器的误差)
    Eigen::Matrix3d I_; // 单位矩阵
};

} // namespace filters
} // namespace uav_vision