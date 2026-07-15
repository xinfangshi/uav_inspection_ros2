cmake_minimum_required(VERSION 3.8)
project(uav_vision)

if(CMAKE_COMPILER_ID STREQUAL "GNU" OR CMAKE_COMPILER_ID STREQUAL "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# ================= 编译核心视觉库 =================
add_library(vision_core SHARED
  src/detectors/defect_detector.cpp         # <-- 算法核心
  src/ros_nodes/camera_subscriber.cpp       # <-- ROS 包装
)
target_include_directories(vision_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(vision_core rclcpp sensor_msgs cv_bridge image_transport std_msgs)
target_link_libraries(vision_core ${OpenCV_LIBS})

# ================= 编译主执行器 =================
add_executable(camera_node src/ros_nodes/vision_main.cpp)   
target_link_libraries(camera_node vision_core)
ament_target_dependencies(camera_node rclcpp)

# ================= 编译独立 Demo =================
add_executable(vision_demo src/demos/vision_demo.cpp)       
target_link_libraries(vision_demo vision_core)

# ================= 安装规则 =================
install(TARGETS vision_core
  DESTINATION lib
)

install(TARGETS camera_node vision_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()