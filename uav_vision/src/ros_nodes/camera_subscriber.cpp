#include "uav_vision/ros_nodes/camera_subscriber.hpp"

namespace uav_vision
{

CameraSubscriber::CameraSubscriber(std::shared_ptr<IDetector> detector) 
    : Node("camera_subscriber_node"), detector_(detector)
{
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 
        rclcpp::SensorDataQoS(),
        std::bind(&CameraSubscriber::image_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "👁️ 视觉节点已启动，携带 AI 大脑等待图像中...");
}

void CameraSubscriber::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;

        if (!frame.empty()) {
            // 🔥 架构之美：直接调用多态接口，不管底层是 OpenCV 还是 TFLite！
            cv::Rect bbox = detector_->detect(frame);

            // 如果发现了缺陷，就在真实传回来的视频流里画个红框！
            if (bbox.area() > 0) {
                cv::rectangle(frame, bbox, cv::Scalar(0, 0, 255), 2);
                cv::putText(frame, "Defect Detected", cv::Point(bbox.x, bbox.y - 10), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
            }

            cv::imshow("UAV Camera Feed (AI Enabled)", frame);
            cv::waitKey(1); 
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ cv_bridge 转换崩溃: %s", e.what());
    }
}

} // namespace uav_vision