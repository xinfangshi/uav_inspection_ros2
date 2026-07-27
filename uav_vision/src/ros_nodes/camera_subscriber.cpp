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

    // 实例化 TF 空间雷达
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
                    // 🛡️ 工业级抗噪测距：取中心 5x5 区域的有效深度平均值！
                    // =========================================================
                    int radius = 2; // 5x5 的邻域
                    float sum_Z = 0.0f;
                    int valid_count = 0;

                    for (int dy = -radius; dy <= radius; ++dy) {
                        for (int dx = -radius; dx <= radius; ++dx) {
                            int nu = std::clamp(u + dx, 0, depth_frame.cols - 1);
                            int nv = std::clamp(v + dy, 0, depth_frame.rows - 1);
                            float val = depth_frame.at<float>(nv, nu);
                        
                            // 只统计有效的物理距离
                            if (std::isfinite(val) && val > 0.1f && val < 50.0f) {
                                sum_Z += val;
                                valid_count++;
                        }
                    }
                }
                    // 计算有效平均值，如果全无效则给个 inf
                    float Z = (valid_count > 0) ? (sum_Z / valid_count) : std::numeric_limits<float>::infinity();
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "🔍 [Debug] 中心 5x5 区域获取有效深度点 %d 个，均值 Z = %f", valid_count, Z);
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

                        // === 空间代数 ===
                        // A. 构造一个在【相机坐标系 (camera_link)】下的 3D 点
                        geometry_msgs::msg::PointStamped pt_cam;
                        pt_cam.header.frame_id = "camera_link";
                        // 🔥 修复 TF 时空穿梭 Bug：将时间戳设为 0，强制获取最新可用的坐标变换！
                        pt_cam.header.stamp.sec = 0;
                        pt_cam.header.stamp.nanosec = 0;
                        
                        // ROS 2 标准相机坐标系：X朝前，Y朝左，Z朝上
                        // 计算机视觉算出的：X朝右，Y朝下，Z朝前
                        pt_cam.point.x = Z;                                // 深度距离就是正前方(X)
                        pt_cam.point.y = -((u - cx) * Z / fx);             // CV的右(X) 是 ROS的左(-Y)
                        pt_cam.point.z = -((v - cy) * Z / fy);             // CV的下(Y) 是 ROS的下(-Z)

                        geometry_msgs::msg::PointStamped pt_map;
                        try {
                            // B. 去 TF 树里查一下此刻无人机的真实姿态，把相机坐标秒变地球绝对坐标！
                            pt_map = tf_buffer_->transform(pt_cam, "map", tf2::durationFromSec(0.1));

                            // 覆盖原来要发出去的消息，现在发的是真实的地球坐标！
                            target_3d_msg.x = pt_map.point.x;
                            target_3d_msg.y = pt_map.point.y;
                            target_3d_msg.z = pt_map.point.z; // 注意，这里我们把 z 放真实的物理高度。不再作标志位了！

                            cv::putText(rgb_frame, "Map X:" + cv::format("%.1f", pt_map.point.x) + " Y:" + cv::format("%.1f", pt_map.point.y), 
                                        cv::Point(bbox.x, bbox.y - 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
                            
                            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "🌍 [绝对定位] 目标处于地球坐标: X=%.2f 米, Y=%.2f 米, 绝对高度 Z=%.2f 米", 
                                pt_map.point.x, pt_map.point.y, pt_map.point.z);
                                
                        } catch (const tf2::TransformException & ex) {
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                "⚠️ 空间坐标树未就绪: %s", ex.what());
                        }

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