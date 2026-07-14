#include <gtest/gtest.h>
#include "uav_control/planners/trajectory_planner.hpp"

TEST(TrajectoryPlannerTest, GenerateTrajectory) {
    uav_control::TrajectoryPlanner planner;
    
    uav_control::Waypoint start = {0.0, 0.0, 0.0};
    uav_control::Waypoint end = {10.0, 5.0, -5.0};
    double T = 2.0;
    double dt = 0.02;

    auto trajectory = planner.generate_trajectory(start, end, T, dt);

    // 1. 断言：生成的轨迹点数量应该符合预期 (T/dt + 1)
    EXPECT_EQ(trajectory.size(), 101);

    // 2. 断言：轨迹的第一个点必须是起点
    EXPECT_NEAR(trajectory.front().x, start.x, 1e-3);
    EXPECT_NEAR(trajectory.front().y, start.y, 1e-3);
    EXPECT_NEAR(trajectory.front().z, start.z, 1e-3);

    // 3. 断言：轨迹的最后一个点必须是终点
    EXPECT_NEAR(trajectory.back().x, end.x, 1e-3);
    EXPECT_NEAR(trajectory.back().y, end.y, 1e-3);
    EXPECT_NEAR(trajectory.back().z, end.z, 1e-3);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}