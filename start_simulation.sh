#!/bin/bash
echo "🚀 正在启动带有摄像头的 PX4 仿真..."

# 🔥 核心免疫补丁：强行将当前脚本的实时线程优先级拉满至 99！
# 彻底免疫 Ubuntu 24.04 的 pthread_mutex_lock 崩溃 Bug！
ulimit -S -r 99
ulimit -H -r 99

# 1. 禁用 Wayland，强制 Gazebo 的 Qt 界面使用 X11
export QT_QPA_PLATFORM=xcb
export WAYLAND_DISPLAY=""

# 2. 强制 Gazebo Ogre2 渲染引擎使用 OpenGL 而不是 Vulkan
export GZ_OGRE2_RENDER_SYSTEM=gl

# 3. 限制 PX4 仿真的加速比（防止物理引擎跑得太快导致锁步崩溃）
export PX4_SIM_SPEED_FACTOR=1

export LIBGL_ALWAYS_SOFTWARE=1
#export HEADLESS=1
cd ~/PX4-Autopilot
# 注意这里改成了 gz_x500_depth！
make px4_sitl gz_x500_depth
