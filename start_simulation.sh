#!/bin/bash
echo "🚀 正在启动 PX4 无头模式 (Headless) 仿真..."
export LIBGL_ALWAYS_SOFTWARE=1
export HEADLESS=1
cd ~/PX4-Autopilot
make px4_sitl gz_x500
