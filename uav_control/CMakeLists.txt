cmake_minimum_required(VERSION 3.8)
project(uav_control)

# ==============================================================================
# 1. 编译器设置 (Compiler Settings)
# ==============================================================================
if(CMAKE_COMPILER_ID STREQUAL "GNU" OR CMAKE_COMPILER_ID STREQUAL "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================================================================
# 2. 查找依赖包 (Dependencies)
# ==============================================================================
find_package(ament_cmake REQUIRED)

# ROS 2 核心依赖
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 第三方依赖: 行为树与数学算法库
find_package(behaviortree_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

# ==============================================================================
# 3. 编译核心动态库 (Core Shared Library)
# ==============================================================================
add_library(bt_nodes SHARED
  src/behavior_trees/takeoff_action.cpp
  src/behavior_trees/goto_waypoint_action.cpp
  src/planners/trajectory_planner.cpp
)

target_include_directories(bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接 ROS 2 原生依赖
ament_target_dependencies(bt_nodes 
  rclcpp 
  px4_msgs 
  behaviortree_cpp
)

# 链接纯 C++ 第三方库 (现代CMake写法：直接链接目标，自动处理头文件路径)
target_link_libraries(bt_nodes
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
  osqp::osqp
)

# ==============================================================================
# 4. 编译可执行文件 (Executables)
# ==============================================================================

# (A) 行为树主执行器
add_executable(tree_executor src/tree_executor.cpp)
target_include_directories(tree_executor PUBLIC include)
target_link_libraries(tree_executor bt_nodes) 
ament_target_dependencies(tree_executor rclcpp px4_msgs behaviortree_cpp ament_index_cpp)

# (B) 纯数学算法独立 Demo
add_executable(planner_demo src/planners/planner_demo.cpp src/planners/trajectory_planner.cpp)
target_include_directories(planner_demo PUBLIC include)
target_link_libraries(planner_demo 
  Eigen3::Eigen
  OsqpEigen::OsqpEigen 
  osqp::osqp
)

# ==============================================================================
# 5. 测试模块 (Testing)
# ==============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trajectory_planner test/test_trajectory_planner.cpp src/planners/trajectory_planner.cpp)
  target_include_directories(test_trajectory_planner PUBLIC include)
  
  # 铁律：避免签名冲突，这里绝不能加 PUBLIC 关键字
  target_link_libraries(test_trajectory_planner 
    Eigen3::Eigen
    OsqpEigen::OsqpEigen 
    osqp::osqp
  )
endif()

# ==============================================================================
# 6. 安装部署规则 (Install Rules)
# ==============================================================================

# 安装动态库 (必须放到公共 lib 目录，Linux 才能找到 .so 链接)
install(TARGETS bt_nodes
  DESTINATION lib
)

# 统一安装所有可执行文件 (放到包的专属目录，ros2 run 才能找到)
install(TARGETS tree_executor planner_demo
  DESTINATION lib/${PROJECT_NAME}
)

# 安装对外暴露的头文件
install(DIRECTORY include/
  DESTINATION include
)

# 统一安装配置文件: XML 剧本和 Launch 启动脚本
install(DIRECTORY behavior_trees launch
  DESTINATION share/${PROJECT_NAME}
)

# ==============================================================================
# 7. Ament 导出 (必须在文件的绝对最后一行！)
# ==============================================================================
ament_package()