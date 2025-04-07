#include "rclcpp/rclcpp.hpp"
#include "hd_radar_driver/hd_radar_driver.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HdRadarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}