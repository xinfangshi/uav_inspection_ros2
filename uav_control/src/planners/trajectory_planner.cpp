#include "uav_control/planners/trajectory_planner.hpp"
#include <iostream>
#include <cmath>

namespace uav_control
{

TrajectoryPlanner::TrajectoryPlanner() {}

std::vector<Waypoint> TrajectoryPlanner::generate_trajectory(const Waypoint& start, const Waypoint& end, double T, double dt)
{
    std::vector<Waypoint> trajectory;
    
    // X, Y, Z 三个维度是互相独立的，我们分别对三个轴求解二次规划
    Eigen::VectorXd cx = solve_minimum_snap_1D(start.x, end.x, T);
    Eigen::VectorXd cy = solve_minimum_snap_1D(start.y, end.y, T);
    Eigen::VectorXd cz = solve_minimum_snap_1D(start.z, end.z, T);

    // 按照频率 (dt = 0.02s) 密集采样，生成飞控需要的真实坐标点
    for (double t = 0; t <= T; t += dt) {
        Waypoint pt;
        // 7阶多项式: p(t) = c0 + c1*t + c2*t^2 + ... + c7*t^7
        Eigen::VectorXd t_vec(8);
        t_vec << 1, t, std::pow(t,2), std::pow(t,3), std::pow(t,4), std::pow(t,5), std::pow(t,6), std::pow(t,7);
        
        pt.x = cx.dot(t_vec);
        pt.y = cy.dot(t_vec);
        pt.z = cz.dot(t_vec);
        trajectory.push_back(pt);
    }
    return trajectory;
}

Eigen::VectorXd TrajectoryPlanner::solve_minimum_snap_1D(double start_p, double end_p, double T)
{
    // Minimum Snap 对应 7 阶多项式，有 8 个系数 (c0 到 c7)
    int num_vars = 8;
    int num_constraints = 8; // 起点和终点各4个状态约束(位置,速度,加速度,Jerk)

    // 1. 构建代价矩阵 Q (目标是 Snap 平方的积分最小化)
    Eigen::SparseMatrix<double> Q(num_vars, num_vars);
    Q.insert(4, 4) = 576.0 * T;
    Q.insert(4, 5) = 1440.0 * std::pow(T, 2); Q.insert(5, 4) = 1440.0 * std::pow(T, 2);
    Q.insert(5, 5) = 4800.0 * std::pow(T, 3);
    Q.insert(4, 6) = 2880.0 * std::pow(T, 3); Q.insert(6, 4) = 2880.0 * std::pow(T, 3);
    Q.insert(6, 6) = 10800.0 * std::pow(T, 5);
    Q.insert(4, 7) = 5040.0 * std::pow(T, 4); Q.insert(7, 4) = 5040.0 * std::pow(T, 4);
    Q.insert(7, 7) = 20160.0 * std::pow(T, 7);
    Q.makeCompressed();

    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(num_vars); // 无一次项代价

    // 2. 构建约束矩阵 A (边界条件)
    Eigen::SparseMatrix<double> A(num_constraints, num_vars);
    // 起点约束 (t=0)
    A.insert(0, 0) = 1.0; // 位置 p(0) = c0
    A.insert(1, 1) = 1.0; // 速度 v(0) = c1
    A.insert(2, 2) = 2.0; // 加速度 a(0) = 2*c2
    A.insert(3, 3) = 6.0; // Jerk j(0) = 6*c3

    // 终点约束 (t=T)
    A.insert(4, 0) = 1.0; A.insert(4, 1) = T; A.insert(4, 2) = std::pow(T,2); A.insert(4, 3) = std::pow(T,3);
    A.insert(4, 4) = std::pow(T,4); A.insert(4, 5) = std::pow(T,5); A.insert(4, 6) = std::pow(T,6); A.insert(4, 7) = std::pow(T,7); // 位置
    A.insert(5, 1) = 1.0; A.insert(5, 2) = 2*T; A.insert(5, 3) = 3*std::pow(T,2); A.insert(5, 4) = 4*std::pow(T,3);
    A.insert(5, 5) = 5*std::pow(T,4); A.insert(5, 6) = 6*std::pow(T,5); A.insert(5, 7) = 7*std::pow(T,6); // 速度
    A.insert(6, 2) = 2.0; A.insert(6, 3) = 6*T; A.insert(6, 4) = 12*std::pow(T,2); A.insert(6, 5) = 20*std::pow(T,3);
    A.insert(6, 6) = 30*std::pow(T,4); A.insert(6, 7) = 42*std::pow(T,5); // 加速度
    A.insert(7, 3) = 6.0; A.insert(7, 4) = 24*T; A.insert(7, 5) = 60*std::pow(T,2); A.insert(7, 6) = 120*std::pow(T,3);
    A.insert(7, 7) = 210*std::pow(T,4); // Jerk
    A.makeCompressed();

    // 3. 构建约束上下界 (Lower bound = Upper bound, 强制严格等于)
    Eigen::VectorXd lowerBound(num_constraints);
    Eigen::VectorXd upperBound(num_constraints);
    // [起点位置, 终点位置, 其余速度加速度全部为0 (平稳悬停)]
    lowerBound << start_p, 0.0, 0.0, 0.0, end_p, 0.0, 0.0, 0.0;
    upperBound = lowerBound;

    // 4. 配置 OSQP 求解器
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false); // 关闭刷屏打印
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(num_vars);
    solver.data()->setNumberOfConstraints(num_constraints);
    
    // 输入数据
    solver.data()->setHessianMatrix(Q);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(A);
    solver.data()->setLowerBound(lowerBound);
    solver.data()->setUpperBound(upperBound);

    // 5. 极速求解！
    solver.initSolver();
    solver.solveProblem();

    // 6. 吐出 8 个完美的系数
    return solver.getSolution();
}

} // namespace uav_control