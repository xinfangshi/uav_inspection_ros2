#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <vector>

namespace uav_control {

// 定义一个 3D 航点结构体
struct Waypoint {
    double x, y, z;
};

// 定义一段轨迹的多项式系数 (X, Y, Z 各 8 个系数，对应 7 阶多项式)
struct TrajectoryCoeffs {
    Eigen::VectorXd x_coeffs;
    Eigen::VectorXd y_coeffs;
    Eigen::VectorXd z_coeffs;
};

class TrajectoryPlanner {
   public:
    TrajectoryPlanner();

    /**
     * @brief 生成从起点到终点的 Minimum Snap 极度平滑轨迹
     * @param start 起点坐标
     * @param end 终点坐标
     * @param T 预计飞行时间 (秒)
     * @param dt 采样步长 (比如 0.02秒，即 50Hz)
     * @return 密集的轨迹点序列
     */
    std::vector<Waypoint> generate_trajectory(const Waypoint& start, const Waypoint& end, double T, double dt = 0.02);

   private:
    // 核心底层算法：基于 OSQP 求解单轴的 Minimum Snap 系数
    Eigen::VectorXd solve_minimum_snap_1D(double start_p, double end_p, double T);
};

}  // namespace uav_control