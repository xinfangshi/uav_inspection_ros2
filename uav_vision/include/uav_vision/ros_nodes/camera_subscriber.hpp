#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <mutex>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "uav_vision/detectors/i_detector.hpp"

#include "uav_vision/filters/kalman_filter.hpp"  // <--- 防抖kalman_filter
#include "uav_vision/filters/lite_sort.hpp" // 引入追踪器

#include <sensor_msgs/msg/camera_info.hpp>

namespace uav_vision
{

class CameraSubscriber : public rclcpp::Node
{
public:
    CameraSubscriber(std::shared_ptr<IDetector> detector);

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
    
    // 广播 3D 空间坐标的发布者
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub_;

    std::shared_ptr<IDetector> detector_;
    
    // 线程安全存取最新的深度图
    cv::Mat latest_depth_frame_;
    std::mutex depth_mutex_;

     // 🔥 新增：卡尔曼滤波器实例
    uav_vision::filters::TargetKalmanFilter kf_;
    uav_vision::filters::LiteSORT tracker_;

    // 专门用于倾听系统 TF 树的空间雷达！
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 监听内参的回调
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // 存放真实的动态内参 (原子操作保证多线程读取安全)
    std::atomic<bool> camera_info_received_;
    std::atomic<float> fx_, fy_, cx_, cy_;
};

} // namespace uav_vision