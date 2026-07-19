#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <geometry_msgs/msg/point.hpp>

// 只引入接口类，不引入具体的实现类
#include "uav_vision/detectors/i_detector.hpp" 

namespace uav_vision
{

class CameraSubscriber : public rclcpp::Node
{
public:
    // 依赖注入：在构造函数里，把一个 AI 大脑 (IDetector) 塞进来！
    CameraSubscriber(std::shared_ptr<IDetector> detector);

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    
    // 存放各种 AI 算法的通用接口指针
    std::shared_ptr<IDetector> detector_;
    // 专门用于广播 AI 识别结果的喇叭
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub_;
};

} // namespace uav_vision