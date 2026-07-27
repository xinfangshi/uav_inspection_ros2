#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

namespace uav_control {
namespace localization {

class TfBroadcaster : public rclcpp::Node
{
public:
    TfBroadcaster();

private:
    void publish_static_camera_transform();
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;

    // 存放从参数服务器读取的机架相机安装偏移量
    double cam_offset_x_;
    double cam_offset_y_;
    double cam_offset_z_;
};

} // namespace localization
} // namespace uav_control