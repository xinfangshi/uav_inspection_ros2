#!/bin/bash
echo "🚀 正在启动带有摄像头的 PX4 仿真..."
export LIBGL_ALWAYS_SOFTWARE=1
export HEADLESS=1
cd ~/PX4-Autopilot
# 注意这里改成了 gz_x500_depth！
make px4_sitl gz_x500_depth
