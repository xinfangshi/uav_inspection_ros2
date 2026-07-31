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

    camera_info_received_ = false;
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info", 10,
        std::bind(&CameraSubscriber::camera_info_callback, this, std::placeholders::_1));

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
                cv::rectangle(rgb_frame, bbox, cv::Scalar(0, 0, 255), 2);
                int u = bbox.x + bbox.width / 2;
                int v = bbox.y + bbox.height / 2;

                if (!depth_frame.empty()) {
                    u = std::clamp(u, 0, depth_frame.cols - 1);
                    v = std::clamp(v, 0, depth_frame.rows - 1);
                    
                    // 🛡️ 工业级抗噪测距：取中心 5x5 区域的有效深度【中位数值】！
                    // =========================================================
                    int radius = 2; // 5x5 的邻域
                    std::vector<float> valid_depths; // 存放所有有效的深度值

                    for (int dy = -radius; dy <= radius; ++dy) {
                        for (int dx = -radius; dx <= radius; ++dx) {
                            int nu = std::clamp(u + dx, 0, depth_frame.cols - 1);
                            int nv = std::clamp(v + dy, 0, depth_frame.rows - 1);
                            float val = depth_frame.at<float>(nv, nu);
                            
                            // 只统计有效的物理距离
                            if (std::isfinite(val) && val > 0.1f && val < 50.0f) {
                                valid_depths.push_back(val);
                            }
                        }
                    }

                    float Z = std::numeric_limits<float>::infinity();

                    if (!valid_depths.empty()) {
                        // 🔥 架构师级升华：使用 std::nth_element 提取中位数！
                        // 相比于排序整个数组，nth_element 的时间复杂度是 O(N)，极其适合边缘计算！
                        size_t n = valid_depths.size() / 2;
                        std::nth_element(valid_depths.begin(), valid_depths.begin() + n, valid_depths.end());
                        Z = valid_depths[n]; // 取出完美的中位数！
                    }

                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "🔍 [Debug] 5x5区域获取有效深度点 %zu 个，中值 Z = %f", valid_depths.size(), Z);

                    
                    if (std::isfinite(Z)) {
                        // 🔥 架构师级防爆锁：如果还没拿到内参，坚决不算 3D 坐标！
                        if (!camera_info_received_) {
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "⏳ 正在等待真实相机内参...");
                            target_3d_msg.z = -1.0;
                        } else {
                        // 动态加载真实的硬件光学参数！
                        float fx = fx_.load();
                        float fy = fy_.load();
                        float cx = cx_.load();
                        float cy = cy_.load();

                        // 原始观测坐标 (带高频抖动噪声)
                        double raw_cam_x = (u - cx) * Z / fx;
                        double raw_cam_y = (v - cy) * Z / fy;
                        double raw_cam_z = Z; 
                        
                        // =========================================================
                        // 🛡️ 卡尔曼滤波介入：洗净噪声！
                        // =========================================================
                        kf_.predict(); // 状态预测

                        if (!kf_.is_initialized()) {
                            kf_.init(raw_cam_x, raw_cam_y, raw_cam_z);
                        } else {
                            kf_.update(raw_cam_x, raw_cam_y, raw_cam_z); // 观测融合
                        }

                        // 取出被洗得极其纯净的最佳坐标
                        Eigen::Vector3d best_state = kf_.get_state();

                        // 空间代数大变身 (使用滤波后的最佳坐标)
                        geometry_msgs::msg::PointStamped pt_cam;
                        pt_cam.header.frame_id = "camera_link";
                        pt_cam.header.stamp.sec = 0;
                        pt_cam.header.stamp.nanosec = 0;
                        
                        // ROS坐标系与相机坐标系转换
                        pt_cam.point.x = best_state.z();
                        pt_cam.point.y = -best_state.x();
                        pt_cam.point.z = -best_state.y();

                        try {
                            // TF2 矩阵绝对映射
                            auto pt_map = tf_buffer_->transform(pt_cam, "map", tf2::durationFromSec(0.1));

                            target_3d_msg.x = pt_map.point.x;
                            target_3d_msg.y = pt_map.point.y;
                            target_3d_msg.z = pt_map.point.z; 

                            cv::putText(rgb_frame, "Map X:" + cv::format("%.2f", pt_map.point.x) + " Y:" + cv::format("%.2f", pt_map.point.y), 
                                        cv::Point(bbox.x, bbox.y - 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
                            
                            // 打印极其稳定的绝对坐标
                            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "🌍 [绝对定位+KF滤波] 目标地球坐标: X=%.2fm, Y=%.2fm, Z=%.2fm", 
                                pt_map.point.x, pt_map.point.y, pt_map.point.z);
                                
                        } catch (const tf2::TransformException & ex) {
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                "⚠️ 空间坐标树未就绪: %s", ex.what());
                            // 🔥🔥🔥 核心修复 1：绝对不允许把 0.0, 0.0 漏出去！！！
                            target_3d_msg.z = -1.0; 
                        }
                    }
                    } else {
                        target_3d_msg.z = -1.0; 
                    }
                } else {
                    target_3d_msg.z = -1.0;
                }
            } else {
                target_3d_msg.z = -1.0;
                // 🔥 如果目标丢失，立刻重置滤波器！防止下次看错！
                kf_.reset();
            }

            target_pub_->publish(target_3d_msg);
            cv::imshow("UAV 3D Vision Feed", rgb_frame);
            cv::waitKey(1);
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ RGB 图像崩溃: %s", e.what());
    }
}

void CameraSubscriber::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (!camera_info_received_) {
        // ROS 2 标准相机内参 K 矩阵是 9个元素的数组: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        fx_ = msg->k[0];
        cx_ = msg->k[2];
        fy_ = msg->k[4];
        cy_ = msg->k[5];
        camera_info_received_ = true;
        
        RCLCPP_INFO(this->get_logger(), "📷 成功获取真实硬件内参! fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                    fx_.load(), fy_.load(), cx_.load(), cy_.load());
    }
}

} // namespace uav_vision