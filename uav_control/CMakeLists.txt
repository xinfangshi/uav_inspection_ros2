cmake_minimum_required(VERSION 3.8)
project(uav_control)

if(CMAKE_COMPILER_ID STREQUAL "GNU" OR CMAKE_COMPILER_ID STREQUAL "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 1. 编译行为树节点库
add_library(bt_nodes SHARED
  src/takeoff_action.cpp
  src/goto_waypoint_action.cpp
)
target_include_directories(bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bt_nodes rclcpp px4_msgs behaviortree_cpp)

# 2. 编译行为树执行器
add_executable(tree_executor src/tree_executor.cpp)
target_include_directories(tree_executor PUBLIC include)
target_link_libraries(tree_executor bt_nodes) 
ament_target_dependencies(tree_executor rclcpp px4_msgs behaviortree_cpp ament_index_cpp)

# ================= 完美版的安装规则 =================

# 1. 动态库 (.so) 必须安装到公共的 lib 目录，Linux 才能链接到它！
install(TARGETS bt_nodes
  DESTINATION lib
)

# 2. 节点引擎 (可执行文件) 必须安装到专属目录，ros2 run 才能找到它！
install(TARGETS tree_executor
  DESTINATION lib/${PROJECT_NAME}
)

# 3. 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 4. 安装 XML 行为树文件
install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

# 5. 安装 Launch 启动脚本 (注意：这里去掉了 launch 后面和 DESTINATION 里的斜杠，彻底解决找不到的 Bug)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 铁律：这行必须在绝对的最后！
ament_package()