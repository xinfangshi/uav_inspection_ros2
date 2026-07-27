#include <rclcpp/rclcpp.hpp>
#include "uav_control/localization/tf_broadcaster.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<uav_control::localization::TfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}