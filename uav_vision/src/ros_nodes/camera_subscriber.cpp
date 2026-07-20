#include "uav_vision/ros_nodes/camera_subscriber.hpp"

namespace uav_vision
{

CameraSubscriber::CameraSubscriber(std::shared_ptr<IDetector> detector) 
    : Node("camera_subscriber_node"), detector_(detector)
{
    // 1. 恢复正确的 QoS (10)，确保能稳定接收桥梁的画面！
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&CameraSubscriber::image_callback, this, std::placeholders::_1));

    depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth_image", 10,
        std::bind(&CameraSubscriber::depth_callback, this, std::placeholders::_1));

    target_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/vision/target_3d_position", 10);

    RCLCPP_INFO(this->get_logger(), "👁️ 视觉与深度 3D 空间解算节点已启动！");
}

void CameraSubscriber::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        std::lock_guard<std::mutex> lock(depth_mutex_);
        latest_depth_frame_ = cv_ptr->image.clone();
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "❌ 深度图转换异常: %s", e.what());
    }
}

void CameraSubscriber::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat rgb_frame = cv_ptr->image;

        if (!rgb_frame.empty()) {
            cv::Rect bbox = detector_->detect(rgb_frame);
            geometry_msgs::msg::Point target_3d_msg;

            cv::Mat depth_frame;
            {
                std::lock_guard<std::mutex> lock(depth_mutex_);
                if (!latest_depth_frame_.empty()) depth_frame = latest_depth_frame_.clone();
            }

            if (bbox.area() > 0) {
                // 🔥 架构解耦：不管深度图有没有拿到，只要看到猎物，先画个大红框再说！
                cv::rectangle(rgb_frame, bbox, cv::Scalar(0, 0, 255), 2);
                int u = bbox.x + bbox.width / 2;
                int v = bbox.y + bbox.height / 2;

                // 如果此时深度图也拿到了，就算 3D 距离
                if (!depth_frame.empty()) {
                    u = std::clamp(u, 0, depth_frame.cols - 1);
                    v = std::clamp(v, 0, depth_frame.rows - 1);
                    // 🔑 核心 1：直接去深度图里查出物理距离 Z (单位：米)
                    float Z = depth_frame.at<float>(v, u);
                    // 打印出底层读到的真实深度值，看看 Gazebo 到底传了什么过来！
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "🔍 [Debug] 当前中心点深度原始值 Z = %f", Z);
                    // 🔥 修复 Bug：去掉了愚蠢的 size_t 转换，放宽到 0.1 米以上即可！
                    if (std::isfinite(Z) && Z > 0.1f && Z < 50.0f) {
                        float fx = 200.0f, fy = 200.0f;
                        float cx = rgb_frame.cols / 2.0f, cy = rgb_frame.rows / 2.0f;

                        target_3d_msg.x = (u - cx) * Z / fx;
                        target_3d_msg.y = (v - cy) * Z / fy;
                        target_3d_msg.z = Z; 

                        // 绿字标注 3D 测距成功！
                        cv::putText(rgb_frame, "3D Target: " + cv::format("%.2fm", Z), 
                                    cv::Point(bbox.x, bbox.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                            "🎯 测算 3D 坐标: X=%.2fm, Y=%.2fm, Z(距离)=%.2fm", target_3d_msg.x, target_3d_msg.y, target_3d_msg.z);
                    } else {
                        target_3d_msg.z = -1.0;
                    }
                } else {
                    target_3d_msg.z = -1.0;
                    cv::putText(rgb_frame, "2D Track Only", cv::Point(bbox.x, bbox.y - 10), 
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
                }
            } else {
                target_3d_msg.z = -1.0;
            }

            target_pub_->publish(target_3d_msg);
            cv::imshow("UAV 3D Vision Feed", rgb_frame);
            cv::waitKey(1);
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ RGB 图像崩溃: %s", e.what());
    }
}

} // namespace uav_vision