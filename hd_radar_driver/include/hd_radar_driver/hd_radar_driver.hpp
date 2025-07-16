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
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "hd_radar_driver/hd_radar_structs.hpp"
#include "hd_radar_driver/modules/hd_radar_pcl.hpp"
#include "hd_radar_driver/modules/hd_radar_raw.hpp"
#include "hd_radar_driver/modules/hd_radar_heat.hpp"
#include "hd_radar_driver/modules/hd_radar_can.hpp"
#include "hd_radar_interfaces/srv/get_raw.hpp"
#include "hd_radar_interfaces/srv/set_thr.hpp"
#include "hd_radar_interfaces/srv/set_vel.hpp"
#include "hd_radar_interfaces/srv/set_mode.hpp"
#include "hd_radar_interfaces/msg/raw.hpp"
#include "hd_radar_interfaces/msg/heat.hpp"

#define UDP_BUFFER_LEN      1500
#define MAX_FRAME_BUFFER    20

using namespace sensor_msgs::msg;
using namespace hd_radar_interfaces::msg;
using namespace hd_radar_interfaces::srv;

class HdRadarNode : public rclcpp::Node 
{
public:
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
    std::string can_device_id_;

    udp_sock_data_t udp_sock_srv_, udp_sock_clnt_;
    can_sock_data_t can_sock_srv_;

    std::mutex mtx_;

    HdRadarPcl hd_radar_pcl_;
    HdRadarRaw hd_radar_raw_;
    HdRadarHeat hd_radar_heat_;
    HdRadarCan hd_radar_can_;

    int32_t udp_server_retval_{0};
    int32_t can_server_retval_{0};

    rclcpp::TimerBase::SharedPtr timer_;

    uint64_t pcl_stat_cnt_{0};
    uint64_t pcl_dyn_cnt_{0};
    uint64_t pcl_stat_c_cnt_{0};
    uint64_t pcl_dyn_c_cnt_{0};
    uint64_t raw_cnt_{0};
    uint64_t heat_cnt_{0};

    // Services
    rclcpp::Service<GetRaw>::SharedPtr srv_get_raw_;
    rclcpp::Service<SetThr>::SharedPtr srv_set_thr_;
    rclcpp::Service<SetVel>::SharedPtr srv_set_vel_;
    rclcpp::Service<SetMode>::SharedPtr srv_set_mode_;

    // Publishers
    rclcpp::Publisher<PointCloud2>::SharedPtr pub_msg_pcl_stat_;
    rclcpp::Publisher<PointCloud2>::SharedPtr pub_msg_pcl_dyn_;
    rclcpp::Publisher<Raw>::SharedPtr pub_msg_raw_;
    rclcpp::Publisher<Heat>::SharedPtr pub_msg_heat_;
    rclcpp::Publisher<Image>::SharedPtr pub_msg_heat_img_;
    rclcpp::Publisher<PointCloud2>::SharedPtr pub_msg_pcl_stat_c_;
    rclcpp::Publisher<PointCloud2>::SharedPtr pub_msg_pcl_dyn_c_;

    void ReadParameters();
    void UdpServerInit();
    void UdpServerStart();
    void UdpClientInit();
    ssize_t UdpClientSend(char * msg, size_t len);
    void CanServerInit();
    void CanServerStart();
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
    void PublishPclStatC();
    void PublishPclDynC();

    void TimerCallback();

    void SrvGetRawClb(
        const std::shared_ptr<GetRaw::Request> request,
        const std::shared_ptr<GetRaw::Response> response);
    void SrvSetThrClb(
        const std::shared_ptr<SetThr::Request> request,
        const std::shared_ptr<SetThr::Response> response);
    void SrvSetVelClb(
        const std::shared_ptr<SetVel::Request> request,
        const std::shared_ptr<SetVel::Response> response);
    void SrvSetModeClb(
        const std::shared_ptr<SetMode::Request> request,
        const std::shared_ptr<SetMode::Response> response);

};

#endif