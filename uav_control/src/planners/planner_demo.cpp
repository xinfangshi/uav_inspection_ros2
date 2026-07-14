#include "uav_control/planners/trajectory_planner.hpp"
#include <iostream>

int main() {
    std::cout << "🚀 [独立 Demo] 启动 Minimum Snap 轨迹规划器测试...\n";
    
    uav_control::TrajectoryPlanner planner;
    uav_control::Waypoint start = {0.0, 0.0, 0.0};
    uav_control::Waypoint end = {10.0, 0.0, 0.0};
    
    std::cout << "🎯 计算从 (0,0,0) 到 (10,0,0) 的平滑轨迹...\n";
    auto trajectory = planner.generate_trajectory(start, end, 2.0, 0.5); // 调大步长方便打印
    
    for (size_t i = 0; i < trajectory.size(); ++i) {
        std::cout << "  点 " << i << ": X=" << trajectory[i].x << "\n";
    }
    
    std::cout << "✅ [独立 Demo] 测试完美结束！\n";
    return 0;
}