#ifndef HD_RADAR_HEAT_HPP
#define HD_RADAR_HEAT_HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/core/mat.hpp>
#include <cv_bridge/cv_bridge.h>
// #include <cmath>

#include "hd_radar_interfaces/msg/heat.hpp"
#include "hd_radar_driver/hd_radar_structs.hpp"
#include "crc16/crc16_ccitt.hpp"

#define IMG_RES 1024.0

class HdRadarHeat
{
public:
    std::string frame_id_;
    bool *ntp_sync_;
    double img_min_val_;
    double img_max_val_;
    double img_pow_;
    double img_max_rng_;
    std::mutex *mtx_;

    // Messages
    sensor_msgs::msg::Image::SharedPtr msg_img_;
    std_msgs::msg::Header msg_img_header_;
    hd_radar_interfaces::msg::Heat msg_heat_hd_;
    
    // Pointer to hd_driver_node
    rclcpp::Node * node_;

    void InitParams(std::string * frame_id, bool * ntp_sync,
                            std::mutex * mtx,  rclcpp::Node * node, 
                            double * img_min_val, double * img_max_val,
                            double * img_pow, double * img_max_rng);

    //Bind callback function
    void BindCallback(std::function<void()> func);

    void ParseHeat(char * buffer, size_t buf_len);

    HdRadarHeat();
    ~HdRadarHeat();

private:
    rclcpp::Time time_stamp_heat_;
    bool check_crc16_;

    // Messages
    udp_msg_heat_t msg_heat_;

    std::vector<uint8_t> heat_full_;
    int64_t heat_cur_frame_{-1};
    int64_t heat_cur_subframe_{0};

    void FillHeat();
    void PublishHeat();
    void ImgPolarToCart(cv::Mat &src, cv::Mat &dst);
    void Meshgrid(const std::vector<double>& x_coords, 
                const std::vector<double>& y_coords,
                std::vector<std::vector<double>> &X, 
                std::vector<std::vector<double>> &Y);

    std::function<void()> PubCallback;
};

#endif