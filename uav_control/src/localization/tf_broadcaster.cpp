#include "uav_control/localization/tf_broadcaster.hpp"

namespace uav_control {
namespace localization {

TfBroadcaster::TfBroadcaster() : Node("tf_broadcaster_node")
{
    // 🔥 架构之美：声明参数并赋予默认值，未来不同机型只需要改 Launch 参数！
    this->declare_parameter<double>("camera_offset_x", 0.1);
    this->declare_parameter<double>("camera_offset_y", 0.0);
    this->declare_parameter<double>("camera_offset_z", 0.0);

    this->get_parameter("camera_offset_x", cam_offset_x_);
    this->get_parameter("camera_offset_y", cam_offset_y_);
    this->get_parameter("camera_offset_z", cam_offset_z_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
        std::bind(&TfBroadcaster::odom_callback, this, std::placeholders::_1));

    // 根据参数发布静态坐标
    publish_static_camera_transform();

    RCLCPP_INFO(this->get_logger(), "🌐 TF 空间树广播中心已启动! 机型相机偏移: [X:%.2f, Y:%.2f, Z:%.2f]", 
                cam_offset_x_, cam_offset_y_, cam_offset_z_);
}

void TfBroadcaster::publish_static_camera_transform()
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_link";   
    t.child_frame_id = "camera_link";  

    // 应用外部注入的参数
    t.transform.translation.x = cam_offset_x_;
    t.transform.translation.y = cam_offset_y_;
    t.transform.translation.z = cam_offset_z_;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    tf_static_broadcaster_->sendTransform(t);
}

void TfBroadcaster::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";         
    t.child_frame_id = "base_link";    

    t.transform.translation.x = msg->position[0];
    t.transform.translation.y = msg->position[1];
    t.transform.translation.z = msg->position[2];

    t.transform.rotation.w = msg->q[0];
    t.transform.rotation.x = msg->q[1];
    t.transform.rotation.y = msg->q[2];
    t.transform.rotation.z = msg->q[3];

    tf_broadcaster_->sendTransform(t);
}

} // namespace localization
} // namespace uav_control