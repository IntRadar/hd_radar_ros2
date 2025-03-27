#ifndef HD_RADAR_DRIVER_HPP
#define HD_RADAR_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/core/mat.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_srvs/srv/set_bool.hpp>

#include <thread>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "hd_radar_driver/hd_radar_structs.hpp"
#include "hd_radar_driver/modules/hd_radar_pcl.hpp"
#include "hd_radar_driver/modules/hd_radar_raw.hpp"
#include "hd_radar_driver/modules/hd_radar_heat.hpp"
#include "hd_radar_interfaces/srv/get_raw.hpp"
#include "hd_radar_interfaces/srv/set_thr.hpp"
#include "hd_radar_interfaces/srv/set_vel.hpp"
#include "hd_radar_interfaces/srv/set_mode.hpp"
#include "hd_radar_interfaces/msg/raw.hpp"
#include "hd_radar_interfaces/msg/heat.hpp"

#define PORT_RECEIVE        25200
#define PORT_SEND           35200
#define BUFFER_LEN          1500
#define MAX_FRAME_BUFFER    20

//Compatible version of radar protocol
#define PROTOCOL_MAJOR 1 // 4 bit format 
#define PROTOCOL_MINOR 1 // 4 bit format 
#define PROTOCOL_VER (((PROTOCOL_MINOR & 0xF) << 4) | (PROTOCOL_MAJOR & 0xF)) // 8 bit format b3:0 major b7:4 minor

class HdRadarNode : public rclcpp::Node 
{
public:
    char buffer_[BUFFER_LEN];

    HdRadarNode();
    ~HdRadarNode();
private:

    std::string topic_;
    std::string frame_id_;
    std::string host_ip_;
    std::string sensor_ip_;
    std::string multicast_ip_;
    uint receive_port_;
    uint send_port_;
    double img_min_val_;
    double img_max_val_;
    bool check_crc16_;
    uint buf_len_;

    sock_data_t sock_server_, sock_client_;

    HdRadarPcl hd_radar_pcl_;
    HdRadarRaw hd_radar_raw_;
    HdRadarHeat hd_radar_heat_;

    // Services
    rclcpp::Service<hd_radar_interfaces::srv::GetRaw>::SharedPtr srv_get_raw_;
    rclcpp::Service<hd_radar_interfaces::srv::SetThr>::SharedPtr srv_set_thr_;
    rclcpp::Service<hd_radar_interfaces::srv::SetVel>::SharedPtr srv_set_vel_;
    rclcpp::Service<hd_radar_interfaces::srv::SetMode>::SharedPtr srv_set_mode_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_msg_pcl_stat_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_msg_pcl_dyn_;
    rclcpp::Publisher<hd_radar_interfaces::msg::Raw>::SharedPtr pub_msg_raw_;
    rclcpp::Publisher<hd_radar_interfaces::msg::Heat>::SharedPtr pub_msg_heat_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_msg_heat_img_;

    void ReadParameters();
    void UdpServerInit();
    void UdpServerStart();
    void UdpClientInit();
    ssize_t UdpClientSend(char * msg, size_t len);
    void RequestRawData(uint8_t frames_to_write);
    void SendThrData(uint16_t sta_threshold, uint16_t sta_azm_sense,
         int16_t sta_rcs_filter, uint16_t dyn_threshold,
         uint16_t dyn_azm_sense, int16_t dyn_rcs_filter);
    void SendVelData(float velocity, uint32_t hold_time);
    void SendModeData(uint8_t mode, uint8_t test_source);
    void PublishPclStat();
    void PublishPclDyn();
    void PublishRaw();
    void PublishHeat();
    void SrvGetRawClb(
        const std::shared_ptr<hd_radar_interfaces::srv::GetRaw::Request> request,
        const std::shared_ptr<hd_radar_interfaces::srv::GetRaw::Response> response);
    void SrvSetThrClb(
        const std::shared_ptr<hd_radar_interfaces::srv::SetThr::Request> request,
        const std::shared_ptr<hd_radar_interfaces::srv::SetThr::Response> response);
    void SrvSetVelClb(
        const std::shared_ptr<hd_radar_interfaces::srv::SetVel::Request> request,
        const std::shared_ptr<hd_radar_interfaces::srv::SetVel::Response> response);
    void SrvSetModeClb(
        const std::shared_ptr<hd_radar_interfaces::srv::SetMode::Request> request,
        const std::shared_ptr<hd_radar_interfaces::srv::SetMode::Response> response);
};

#endif