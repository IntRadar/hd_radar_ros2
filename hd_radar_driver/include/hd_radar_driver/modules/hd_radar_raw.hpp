#ifndef HD_RADAR_RAW_HPP
#define HD_RADAR_RAW_HPP

#include <rclcpp/rclcpp.hpp>
#include "hd_radar_interfaces/msg/raw.hpp"
#include "hd_radar_driver/hd_radar_structs.hpp"
#include "crc16/crc16_ccitt.hpp"

class HdRadarRaw
{
public:
    std::string frame_id_;
    bool *ntp_sync_;
    std::mutex *mtx_;

    // Messages
    hd_radar_interfaces::msg::Raw msg_raw_hd_;

    // Pointer to hd_driver_node
    rclcpp::Node * node_;
    
    void InitParams(std::string * frame_id, bool * check_crc16,
                            bool * ntp_sync, std::mutex * mtx,
                            rclcpp::Node * node);

    //Bind callback function
    void BindCallback(std::function<void()> func);

    void ParseRaw(char * buffer, size_t buf_len);

    HdRadarRaw();
    ~HdRadarRaw();

private:
    bool check_crc16_;
    // Messages
    udp_msg_raw_t msg_raw_;

    int64_t raw_cur_frame_{0};
    int64_t raw_cur_subframe_{0};

    void PublishRaw();

    std::function<void()> PubCallback;
};

#endif