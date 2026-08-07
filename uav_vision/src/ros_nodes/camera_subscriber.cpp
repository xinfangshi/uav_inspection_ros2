#include "uav_vision/ros_nodes/camera_subscriber.hpp"

namespace uav_vision
{

CameraSubscriber::CameraSubscriber(std::shared_ptr<IDetector> detector) 
    : Node("camera_subscriber_node"), detector_(detector), camera_info_received_(false)
{
    // 初始化 RGB 彩色图像与深度图像的订阅者
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&CameraSubscriber::image_callback, this, std::placeholders::_1));

    depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth_image", rclcpp::SensorDataQoS(),
        std::bind(&CameraSubscriber::depth_callback, this, std::placeholders::_1));

    // 初始化相机内参订阅者
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info", 10,
        std::bind(&CameraSubscriber::camera_info_callback, this, std::placeholders::_1));

    // 初始化已巡检目标坐标（空间黑名单）订阅者
    rclcpp::QoS inspected_qos(10);
    inspected_qos.transient_local();
    inspected_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/vision/inspected_target", inspected_qos,
        std::bind(&CameraSubscriber::inspected_callback, this, std::placeholders::_1));

    // 初始化主目标 3D 坐标发布者
    target_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/vision/target_3d_position", 10);

    // 初始化 TF2 变换监听器与缓冲区
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // =========================================================
    // 📸 初始化巡检报告存放目录 (存放于用户的 Home 目录下)
    // =========================================================
    const char* home_dir = std::getenv("HOME");
    if (home_dir) {
        report_dir_ = std::string(home_dir) + "/uav_inspection_reports/";
        std::filesystem::create_directories(report_dir_); // 目录不存在则自动创建
    }
    trigger_snapshot_ = false; // 初始化快门为关闭状态

    RCLCPP_INFO(this->get_logger(), "👁️ 视觉订阅主节点已成功启动。");
}

void CameraSubscriber::inspected_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    // 将新巡检完毕的目标追加至空间黑名单
    std::lock_guard<std::mutex> lock(memory_mutex_);
    inspected_targets_.push_back(Eigen::Vector3d(msg->x, msg->y, msg->z));
    RCLCPP_INFO(this->get_logger(), "🧠 [空间记忆] 目标 (X:%.2f, Y:%.2f, Z:%.2f) 已追加至视觉黑名单。", msg->x, msg->y, msg->z);

    // 🔥 核心事件驱动：接收到控制端的完成信号，立即触发快门标志位！
    trigger_snapshot_ = true;
}

void CameraSubscriber::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (!camera_info_received_) {
        // 从 K 矩阵中提取相机的内参 (焦距与光心)
        fx_ = msg->k[0];
        cx_ = msg->k[2];
        fy_ = msg->k[4];
        cy_ = msg->k[5];
        camera_info_received_ = true;
        RCLCPP_INFO(this->get_logger(), "📷 [相机内参] 参数加载完成: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                    fx_.load(), fy_.load(), cx_.load(), cy_.load());
    }
}

void CameraSubscriber::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        // 深度图采用 32位浮点编码 (32FC1)，单位为米
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

        if (rgb_frame.empty()) return;

        // 执行 2D 目标检测算法
        std::vector<cv::Rect> bboxes = detector_->detect(rgb_frame);
        geometry_msgs::msg::Point target_3d_msg;
        target_3d_msg.z = -1.0; // 默认状态：目标未发现或无效

        // 使用 Lite-SORT 进行多目标追踪 (MOT) 与数据关联
        std::vector<filters::TrackedTarget> tracks = tracker_.update(bboxes);

        // 线程安全地获取最新深度帧
        cv::Mat depth_frame;
        {
            std::lock_guard<std::mutex> lock(depth_mutex_);
            if (!latest_depth_frame_.empty()) depth_frame = latest_depth_frame_.clone();
        }

        // 主目标选取变量 (优先级：未被拉黑 且 最接近图像中心)
        int primary_id = -1;
        cv::Rect primary_bbox(0, 0, 0, 0);
        double min_dist_to_center = std::numeric_limits<double>::max();
        
        // 🔥 修复：这里用来暂存的，必须是真实的 MAP 地球坐标，绝不能是相机坐标！
        double best_raw_map_x = 0.0, best_raw_map_y = 0.0, best_raw_map_z = 0.0;

        if (camera_info_received_ && !depth_frame.empty()) {
            float fx = fx_.load(), fy = fy_.load(), cx = cx_.load(), cy = cy_.load();

            for (const auto& trk : tracks) {
                // 1. 获取目标在高清 RGB 原图 (1920x1080) 上的真实中心像素
                int u = trk.bbox.x + trk.bbox.width / 2;
                int v = trk.bbox.y + trk.bbox.height / 2;

                // 🔥 核心修复 1：保护原始高清坐标，绝不能用 depth_frame 去截断它！
                u = std::clamp(u, 0, rgb_frame.cols - 1);
                v = std::clamp(v, 0, rgb_frame.rows - 1);

                // =========================================================
                // 🔥 核心修复 2：计算 1080p 到 480p 的空间降维缩放比例
                // =========================================================
                double scale_x = static_cast<double>(depth_frame.cols) / rgb_frame.cols;
                double scale_y = static_cast<double>(depth_frame.rows) / rgb_frame.rows;

                // 映射出在深度图 (640x480) 上的实际取样坐标
                int depth_u = static_cast<int>(u * scale_x);
                int depth_v = static_cast<int>(v * scale_y);

                // 限制深度坐标点在图像边界内，防止数组越界
                depth_u = std::clamp(depth_u, 0, depth_frame.cols - 1);
                depth_v = std::clamp(depth_v, 0, depth_frame.rows - 1);

                // 🛡️ 工业级抗噪测距：取中心 5x5 区域的有效深度【中位数值】
                int radius = 2; 
                std::vector<float> valid_depths;

                for (int dy = -radius; dy <= radius; ++dy) {
                    for (int dx = -radius; dx <= radius; ++dx) {
                        // 🔥 核心修复 3：查深度必须使用缩小映射后的 depth_u 和 depth_v！
                        int nu = std::clamp(depth_u + dx, 0, depth_frame.cols - 1);
                        int nv = std::clamp(depth_v + dy, 0, depth_frame.rows - 1);
                        float val = depth_frame.at<float>(nv, nu);
                        
                        if (std::isfinite(val) && val > 0.1f && val < 50.0f) {
                            valid_depths.push_back(val);
                        }
                    }
                }

                float Z = std::numeric_limits<float>::infinity();

                if (!valid_depths.empty()) {
                    size_t n = valid_depths.size() / 2;
                    std::nth_element(valid_depths.begin(), valid_depths.begin() + n, valid_depths.end());
                    Z = valid_depths[n]; 
                }

                if (std::isfinite(Z)) {
                    // =========================================================
                    // 🔥 核心修复 4：针孔逆投影，必须用回原始的 1080p 坐标 (u, v) 
                    // 以及 1080p 的相机内参 (cx, cy, fx, fy)！
                    // =========================================================
                    double raw_opt_x = (u - cx) * Z / fx;
                    double raw_opt_y = (v - cy) * Z / fy;
                    double raw_opt_z = Z;
                    // =========================================================
                    // 📍 探针 1：针孔反投影数据 (检查原始深度和相机局部坐标)
                    // =========================================================
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "🔍 [探针1 - 局部解算] 目标像素(u:%d, v:%d) | 原始深度 Z: %.2fm | 光学坐标(X:%.2f, Y:%.2f, Z:%.2f)",
                    u, v, Z, raw_opt_x, raw_opt_y, raw_opt_z);

                    // 构造 PointStamped 消息用于 TF2 坐标变换
                    geometry_msgs::msg::PointStamped pt_opt;
                    pt_opt.header.frame_id = "camera_optical_link"; 
                    // // 产生的微小时间差跳动，将全部交由下方的卡尔曼滤波器 (KF) 去碾碎平滑！
                    // pt_opt.header.stamp.sec = 0;
                    // pt_opt.header.stamp.nanosec = 0;
                    // ✅ 替换为这一行 (让 TF 时光倒流，回溯到拍照那一瞬间的飞机 Pitch 倾角)：
                    pt_opt.header.stamp = msg->header.stamp;
                    
                    // 🔥 核心重构 2：不用再手工交换轴向和加负号了！原汁原味地塞进去！
                    pt_opt.point.x = raw_opt_x;
                    pt_opt.point.y = raw_opt_y;
                    pt_opt.point.z = raw_opt_z; 

                    try {
                        // =========================================================
                        // 🔥 架构巅峰：调用 TF2 矩阵绝对映射！
                        // 它会自动完成 光学->物理->机身(ENU)->地球(Map) 的四级跳跃！
                        // =========================================================
                        // 📍 探针 2：抓取 TF 树那一瞬间的真实姿态 (极其致命的一环)
                        // =========================================================
                        geometry_msgs::msg::TransformStamped cam_tf = tf_buffer_->lookupTransform(
                            "map", "camera_optical_link", pt_opt.header.stamp, tf2::durationFromSec(0.05));
                        
                        // 把四元数转成人类能看懂的欧拉角 (度数)
                        tf2::Quaternion q(
                            cam_tf.transform.rotation.x, cam_tf.transform.rotation.y, 
                            cam_tf.transform.rotation.z, cam_tf.transform.rotation.w);
                        tf2::Matrix3x3 m(q);
                        double roll, pitch, yaw;
                        m.getRPY(roll, pitch, yaw);
                        
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "🌐 [探针2 - TF姿态] TF树认为当时相机的位置: (X:%.2f, Y:%.2f, Z:%.2f) | 朝向角度: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°",
                            cam_tf.transform.translation.x, cam_tf.transform.translation.y, cam_tf.transform.translation.z,
                            roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);

                        // =========================================================
                        // 📍 探针 3：执行映射并输出最终的原始 MAP 坐标
                        // =========================================================
                        auto pt_map_raw = tf_buffer_->transform(pt_opt, "map", tf2::durationFromSec(0.05));

                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "🎯 [探针3 - 绝对映射] 射线打在地图上的原始坐标: X=%.2f, Y=%.2f, Z=%.2f",
                            pt_map_raw.point.x, pt_map_raw.point.y, pt_map_raw.point.z);

                        // 空间黑名单校验 (防重影去重)
                        bool is_blacklisted = false;
                        {
                            std::lock_guard<std::mutex> lock(memory_mutex_);
                            for (const auto& inspected_pt : inspected_targets_) {
                                // 用真实的地球坐标进行欧式距离比对
                                double dist = std::sqrt(std::pow(pt_map_raw.point.x - inspected_pt.x(), 2) +
                                                        std::pow(pt_map_raw.point.y - inspected_pt.y(), 2) +
                                                        std::pow(pt_map_raw.point.z - inspected_pt.z(), 2));
                                if (dist < 3.0) { 
                                    is_blacklisted = true;
                                    break;
                                }
                            }
                        }

                        // 渲染已巡检目标 (灰色)
                        if (is_blacklisted) {
                            cv::rectangle(rgb_frame, trk.bbox, cv::Scalar(128, 128, 128), 2);
                            cv::putText(rgb_frame, "Inspected", cv::Point(trk.bbox.x, trk.bbox.y - 10), 
                                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(128, 128, 128), 2);
                            continue; 
                        }

                        // 渲染备选目标 (黄色)
                        cv::rectangle(rgb_frame, trk.bbox, cv::Scalar(0, 255, 255), 2);
                        cv::putText(rgb_frame, "ID: " + std::to_string(trk.id), cv::Point(trk.bbox.x, trk.bbox.y - 10), 
                                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

                        // 目标优先级策略：选取距离图像中心最近的目标
                        double dist_to_center = std::sqrt(std::pow(u - cx, 2) + std::pow(v - cy, 2));
                        if (dist_to_center < min_dist_to_center) {
                            min_dist_to_center = dist_to_center;
                            primary_id = trk.id;
                            primary_bbox = trk.bbox;
                            
                            // 🔥 存储原生的地球绝对坐标，准备喂给卡尔曼！
                            best_raw_map_x = pt_map_raw.point.x;
                            best_raw_map_y = pt_map_raw.point.y;
                            best_raw_map_z = pt_map_raw.point.z;
                        }

                    } catch (const tf2::TransformException & ex) {
                        continue; 
                    }
                }
            }

            // 对优先级最高的主目标进行状态估计与数据广播
            if (primary_id != -1) {
                cv::rectangle(rgb_frame, primary_bbox, cv::Scalar(0, 0, 255), 3);
                cv::putText(rgb_frame, "LOCKED", cv::Point(primary_bbox.x, primary_bbox.y + primary_bbox.height + 20), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);

                // =========================================================
                // 🛡️ 卡尔曼滤波介入：在绝对地球坐标系 (MAP Frame) 下洗净噪声！
                // 此时目标对于地图是绝对静止的，完全符合 KF 的动力学模型！
                // =========================================================
                kf_.predict(); 

                if (!kf_.is_initialized()) {
                    kf_.init(best_raw_map_x, best_raw_map_y, best_raw_map_z);
                } else {
                    // 提取 KF 当前的稳定状态
                    Eigen::Vector3d current_state = kf_.get_state();
                                
                // 计算新来的观测坐标，和当前稳定坐标的跳变距离
                    double jump_dist = std::sqrt(std::pow(best_raw_map_x - current_state.x(), 2) +
                                       std::pow(best_raw_map_y - current_state.y(), 2) +
                                       std::pow(best_raw_map_z - current_state.z(), 2));

                    // 🔥 核心修复 2：如果一帧之内，坐标跳变超过 2.5 米，绝对是飞机急刹车引起的物理畸变！
                    // 直接丢弃这个脏数据，拒绝融合！保护 KF 状态的纯洁性！
                    if (jump_dist < 2.0) {
                        kf_.update(best_raw_map_x, best_raw_map_y, best_raw_map_z); 
                    } else {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                            "🛡️ [KF 门控] 检测到姿态突变引发的坐标瞬移 (跳变 %.2fm)，强行剔除脏数据！", jump_dist);
                    }
                }

                // 取出被洗得极其纯净的最佳地球绝对坐标
                Eigen::Vector3d best_state_map = kf_.get_state();
                // 🔥 核心重构 3：已经是地球坐标了，直接拿来用，不需要转来转去！
                target_3d_msg.x = best_state_map.x();
                target_3d_msg.y = best_state_map.y();
                target_3d_msg.z = best_state_map.z(); 

                cv::putText(rgb_frame, "Map X:" + cv::format("%.2f", target_3d_msg.x) + " Y:" + cv::format("%.2f", target_3d_msg.y), 
                            cv::Point(primary_bbox.x, primary_bbox.y - 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "[定位] 主目标 (ID:%d) 地球坐标: X=%.2fm, Y=%.2fm, Z=%.2fm", 
                    primary_id, target_3d_msg.x, target_3d_msg.y, target_3d_msg.z);

            } else {
                target_3d_msg.z = -1.0;
                kf_.reset(); // 目标丢失或切换时，重置卡尔曼滤波器状态
            }

            // 📸 证据落盘 (保持异步存图原样不动)
            if (trigger_snapshot_) {
                trigger_snapshot_ = false; 
                auto now = std::chrono::system_clock::now();
                std::time_t now_time = std::chrono::system_clock::to_time_t(now);
                char time_str[100];
                std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", std::localtime(&now_time));
                std::string filename = report_dir_ + "Defect_" + time_str + ".jpg";
                
                cv::Mat frame_to_save = rgb_frame.clone();
                std::thread([filename, frame_to_save, this]() {
                    cv::imwrite(filename, frame_to_save);
                    RCLCPP_INFO(this->get_logger(), "💾 [异步存盘] 缺陷图像已安全落地: %s", filename.c_str());
                }).detach();
            }

            target_pub_->publish(target_3d_msg);
            cv::imshow("UAV 3D Vision Feed", rgb_frame);
            cv::waitKey(1);
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV Bridge 异常: %s", e.what());
    }
}

} // namespace uav_vision