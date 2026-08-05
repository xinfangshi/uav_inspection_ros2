#include "uav_control/localization/tf_broadcaster.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace uav_control {
namespace localization {

TfBroadcaster::TfBroadcaster() : Node("tf_broadcaster_node")
{
    this->declare_parameter<double>("camera_offset_x", 0.15);
    this->declare_parameter<double>("camera_offset_y", 0.0);
    this->declare_parameter<double>("camera_offset_z", -0.05);

    this->get_parameter("camera_offset_x", cam_offset_x_);
    this->get_parameter("camera_offset_y", cam_offset_y_);
    this->get_parameter("camera_offset_z", cam_offset_z_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
        std::bind(&TfBroadcaster::odom_callback, this, std::placeholders::_1));

    // 启动时，一次性发布相机物理外壳和光学镜头的相对位置！
    publish_static_camera_transform();

    RCLCPP_INFO(this->get_logger(), "🌐 TF 空间树广播中心已启动! 坐标系 NED -> ENU 自动转换已开启！");
}

void TfBroadcaster::publish_static_camera_transform()
{
    // =========================================================
    // 🌳 树枝 1：无人机中心 (base_link) -> 相机外壳 (camera_link)
    // =========================================================
    geometry_msgs::msg::TransformStamped t_cam;
    t_cam.header.stamp = this->get_clock()->now();
    t_cam.header.frame_id = "base_link";   
    t_cam.child_frame_id = "camera_link";  
    t_cam.transform.translation.x = cam_offset_x_;
    t_cam.transform.translation.y = cam_offset_y_;
    t_cam.transform.translation.z = cam_offset_z_;
    t_cam.transform.rotation.x = 0.0;
    t_cam.transform.rotation.y = 0.0;
    t_cam.transform.rotation.z = 0.0;
    t_cam.transform.rotation.w = 1.0;

    // =========================================================
    // 🌳 树枝 2：相机外壳 (camera_link) -> OpenCV 光学坐标系 (camera_optical_link)
    // 🔥 架构核心：ROS 标准是 FLU(前左上)，OpenCV 标准是 RDF(右下前)
    // 必须让 TF 树自己扭转过来！(Roll = -90度, Pitch = 0, Yaw = -90度)
    // =========================================================
    geometry_msgs::msg::TransformStamped t_opt;
    t_opt.header.stamp = this->get_clock()->now();
    t_opt.header.frame_id = "camera_link";
    t_opt.child_frame_id = "camera_optical_link";
    t_opt.transform.translation.x = 0.0;
    t_opt.transform.translation.y = 0.0;
    t_opt.transform.translation.z = 0.0;

    tf2::Quaternion q_opt;
    // 使用欧拉角生成四元数：(-pi/2, 0, -pi/2)
    q_opt.setRPY(-M_PI/2.0, 0.0, -M_PI/2.0);
    t_opt.transform.rotation.x = q_opt.x();
    t_opt.transform.rotation.y = q_opt.y();
    t_opt.transform.rotation.z = q_opt.z();
    t_opt.transform.rotation.w = q_opt.w();

    // 广播这两根静态树枝！
    tf_static_broadcaster_->sendTransform({t_cam, t_opt});
}

void TfBroadcaster::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";         
    t.child_frame_id = "base_link";    

    // 平移：NED 转 ENU
    t.transform.translation.x = msg->position[1];  // East
    t.transform.translation.y = msg->position[0];  // North
    t.transform.translation.z = -msg->position[2]; // Up

    // 🔥 史诗级数学修复：四元数矩阵连乘！绝不手工交换！
    tf2::Quaternion q_ned_to_frd(msg->q[1], msg->q[2], msg->q[3], msg->q[0]); // x,y,z,w
    
    tf2::Quaternion q_enu_to_ned;
    q_enu_to_ned.setRPY(M_PI, 0.0, -M_PI/2.0); 

    tf2::Quaternion q_frd_to_flu;
    q_frd_to_flu.setRPY(M_PI, 0.0, 0.0);

    // 连乘组合：ENU -> NED -> FRD -> FLU
    //四元数乘法 q1 * q2 表示先施加 q2 的旋转，再施加 q1 的旋转
    tf2::Quaternion q_enu_to_flu = q_frd_to_flu * q_ned_to_frd * q_enu_to_ned;
    q_enu_to_flu.normalize();

    t.transform.rotation.x = q_enu_to_flu.x();
    t.transform.rotation.y = q_enu_to_flu.y();
    t.transform.rotation.z = q_enu_to_flu.z();
    t.transform.rotation.w = q_enu_to_flu.w();

    tf_broadcaster_->sendTransform(t);
}

} // namespace localization
} // namespace uav_control