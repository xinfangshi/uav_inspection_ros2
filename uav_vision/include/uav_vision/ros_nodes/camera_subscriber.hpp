#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <mutex>

#include "uav_vision/detectors/i_detector.hpp"

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
};

} // namespace uav_vision