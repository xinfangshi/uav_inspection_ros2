#include "uav_vision/ros_nodes/camera_subscriber.hpp"

namespace uav_vision
{

CameraSubscriber::CameraSubscriber(std::shared_ptr<IDetector> detector) 
    : Node("camera_subscriber_node"), detector_(detector)
{
    // 1. 创建接收画面的耳朵
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 
        10, 
        std::bind(&CameraSubscriber::image_callback, this, std::placeholders::_1)
    );
    
    // 2. 🔥 架构升级：创建向外广播 AI 结果的喇叭！
    target_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/vision/target_position", 10);
    
    RCLCPP_INFO(this->get_logger(), "👁️ 视觉节点已启动，携带 AI 大脑等待图像中...");
}

void CameraSubscriber::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;

        if (!frame.empty()) {
            // 调用多态 AI 接口
            cv::Rect bbox = detector_->detect(frame);
            
            // 准备广播消息
            geometry_msgs::msg::Point target_msg;

            if (bbox.area() > 0) {
                // 计算目标在画面中的中心像素坐标
                target_msg.x = bbox.x + bbox.width / 2.0;
                target_msg.y = bbox.y + bbox.height / 2.0;
                target_msg.z = 1.0; // 状态位 z=1.0 表示“锁定目标”

                cv::rectangle(frame, bbox, cv::Scalar(0, 0, 255), 2);
                cv::putText(frame, "Target Locked", cv::Point(bbox.x, bbox.y - 10), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
                            
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "🎯 锁定目标！广播像素坐标: X=%.1f, Y=%.1f", target_msg.x, target_msg.y);
            } else {
                // 如果没看到目标，中心点无意义，状态位 z=0.0 表示“未发现”
                target_msg.x = -1.0;
                target_msg.y = -1.0;
                target_msg.z = 0.0; 
            }
            
            // 🚀 向全网广播这个高价值的 AI 结果！
            target_pub_->publish(target_msg);

            cv::imshow("UAV Camera Feed (AI Enabled)", frame);
            cv::waitKey(1); 
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ cv_bridge 转换崩溃: %s", e.what());
    }
}

} // namespace uav_vision