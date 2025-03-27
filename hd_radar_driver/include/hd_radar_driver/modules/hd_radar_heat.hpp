#ifndef HD_RADAR_HEAT_HPP
#define HD_RADAR_HEAT_HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/core/mat.hpp>
#include <cv_bridge/cv_bridge.h>

#include "hd_radar_interfaces/msg/heat.hpp"
#include "hd_radar_driver/hd_radar_structs.hpp"
#include "crc16/crc16_ccitt.hpp"

class HdRadarHeat
{
public:
    std::string frame_id_;
    double img_min_val_;
    double img_max_val_;

    // Messages
    sensor_msgs::msg::Image::SharedPtr msg_img_;
    hd_radar_interfaces::msg::Heat msg_heat_hd_;

    // Pointer to hd_driver_node
    rclcpp::Node * node_;

    void InitParams(std::string * frame_id,  rclcpp::Node * node, 
                double * img_min_val_, double * img_max_val_);

    //Bind callback function
    void BindCallback(std::function<void()> func);

    void ParseHeat(char * buffer, size_t buf_len);

    HdRadarHeat();
    ~HdRadarHeat();
private:
    rclcpp::Time time_stamp_heat_;
    bool check_crc16_;

    // Messages
    msg_heat_t msg_heat_;

    std::vector<float> heat_full_;
    int64_t heat_cur_frame_{-1};
    int64_t heat_cur_subframe_{0};

    void FillHeat();
    void PublishHeat();

    std::function<void()> PubCallback;
};

#endif