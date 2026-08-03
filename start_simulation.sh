#!/bin/bash
echo "🚀 正在启动带有摄像头的 PX4 仿真..."

# 🔥 核心免疫补丁：强行将当前脚本的实时线程优先级拉满至 99！
# 彻底免疫 Ubuntu 24.04 的 pthread_mutex_lock 崩溃 Bug！
ulimit -S -r 99
ulimit -H -r 99

export LIBGL_ALWAYS_SOFTWARE=1
#export HEADLESS=1
cd ~/PX4-Autopilot
# 注意这里改成了 gz_x500_depth！
make px4_sitl gz_x500_depth
