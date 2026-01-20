#ifndef HD_RADAR_PCL_HPP
#define HD_RADAR_PCL_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "hd_radar_driver/hd_radar_structs.hpp"
#include "crc16/crc16_ccitt.hpp"

using namespace sensor_msgs::msg;

class HdRadarPcl
{
public:
    std::string frame_id_;

    // Messages
    sensor_msgs::msg::PointCloud2 msg_pcl2_;

    // Pointer to hd_driver_node
    rclcpp::Node * node_;

    void InitParams(std::string * frame_id, bool * check_crc16, 
                            bool * ntp_sync, std::mutex * mtx,
                            rclcpp::Node * node);

    //Bind callback function
    void BindStatCallback(std::function<void()> func);
    void BindDynCallback(std::function<void()> func);

    void ParsePcl(char * buffer, size_t buf_len);

    HdRadarPcl();
    ~HdRadarPcl();

private:
    bool check_crc16_;
    bool *ntp_sync_;
    uint32_t pcl_pnt_received_{0};
    int64_t pcl_cur_frame_{-1};
    int64_t pcl_cur_cloud_{-1};
    std::vector<udp_pcl_data_t> pcl_pnts_in_frame_;
    std::mutex *mtx_;
    // uint64_t time_stamp_radar_prev_{0};
        
    // Messages
    udp_msg_pcl_t msg_pcl_;

    // Publishers
    rclcpp::Publisher<PointCloud2>::SharedPtr pub_msg_pcl_stat_;
    rclcpp::Publisher<PointCloud2>::SharedPtr pub_msg_pcl_dyn_;

    void PublishPcl();
    void FillPcl();
    std::function<void()> PubStatCallback;
    std::function<void()> PubDynCallback;
};

#endif