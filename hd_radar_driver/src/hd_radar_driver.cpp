
#include "hd_radar_driver/hd_radar_driver.hpp"
 
void HdRadarNode::ReadParameters()
{
    this->declare_parameter("topic", "hd_radar");
    topic_ = this->get_parameter("topic").as_string();

    this->declare_parameter("frame_id", "hd_radar");
    frame_id_ = this->get_parameter("frame_id").as_string();

    this->declare_parameter("host_ip", "192.168.1.100");
    host_ip_ = this->get_parameter("host_ip").as_string();

    this->declare_parameter("sensor_ip", "192.168.1.50");
    sensor_ip_ = this->get_parameter("sensor_ip").as_string();

    this->declare_parameter("multicast_ip", "239.168.1.50");
    multicast_ip_ = this->get_parameter("multicast_ip").as_string();

    this->declare_parameter("receive_port", 2500);
    receive_port_ = this->get_parameter("receive_port").as_int();

    this->declare_parameter("send_port", 3500);
    send_port_ = this->get_parameter("send_port").as_int();

    this->declare_parameter("img_min_val", 0.0);
    img_min_val_ = this->get_parameter("img_min_val").as_double();

    this->declare_parameter("img_max_val", 120.0);
    img_max_val_ = this->get_parameter("img_max_val").as_double();

    this->declare_parameter("img_pow", 4.0);
    img_pow_ = this->get_parameter("img_pow").as_double();

    this->declare_parameter("img_max_rng", 25.0);
    img_max_rng_ = this->get_parameter("img_max_rng").as_double();

    this->declare_parameter("check_crc16", false);
    check_crc16_ = this->get_parameter("check_crc16").as_bool();

    this->declare_parameter("can_device_id", "can0");
    can_device_id_ = this->get_parameter("can_device_id").as_string();
}

void HdRadarNode::PublishPclStat() 
{
    RCLCPP_DEBUG(this->get_logger(), "UDP: Pcl static frame collected");
    mtx_.lock();
    pcl_stat_cnt_++;
    mtx_.unlock();
    pub_msg_pcl_stat_->publish(hd_radar_pcl_.msg_pcl2_);
}

void HdRadarNode::PublishPclDyn() 
{
    RCLCPP_DEBUG(this->get_logger(), "UDP: Pcl dynamic frame collected");
    mtx_.lock();
    pcl_dyn_cnt_++;
    mtx_.unlock();
    pub_msg_pcl_dyn_->publish(hd_radar_pcl_.msg_pcl2_);
}

void HdRadarNode::PublishPclStatC() 
{
    RCLCPP_DEBUG(this->get_logger(), "CAN: Pcl static frame collected");
    mtx_.lock();
    pcl_stat_c_cnt_++;
    mtx_.unlock();
    pub_msg_pcl_stat_c_->publish(hd_radar_can_.msg_pcl2_);
}

void HdRadarNode::PublishPclDynC() 
{
    RCLCPP_DEBUG(this->get_logger(), "CAN: Pcl dynamic frame collected");
    mtx_.lock();
    pcl_dyn_c_cnt_++;
    mtx_.unlock();
    pub_msg_pcl_dyn_c_->publish(hd_radar_can_.msg_pcl2_);
}

void HdRadarNode::PublishRaw() 
{
    RCLCPP_DEBUG(this->get_logger(), "UDP: Raw frame collected");
    mtx_.lock();
    raw_cnt_++;
    mtx_.unlock();
    pub_msg_raw_->publish(hd_radar_raw_.msg_raw_hd_);
}

void HdRadarNode::PublishHeat()
{
    RCLCPP_DEBUG(this->get_logger(), "UDP: Heat frame collected");
    mtx_.lock();
    heat_cnt_++;
    mtx_.unlock();
    pub_msg_heat_->publish(hd_radar_heat_.msg_heat_hd_);
    pub_msg_heat_img_->publish(*hd_radar_heat_.msg_img_);
}

void HdRadarNode::SrvGetRawClb(
    const std::shared_ptr<hd_radar_interfaces::srv::GetRaw::Request> request,
    const std::shared_ptr<hd_radar_interfaces::srv::GetRaw::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "SERVICE: Get raw");
    if (request->frames_num <= MAX_FRAME_BUFFER) {
        response->success = true;
        RequestRawData(request->frames_num);
    }
    else if (request->frames_num == 0) {
        RCLCPP_ERROR(this->get_logger(), "SERVICE: Minimum frame number is 1");
        response->success = false;
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "SERVICE: Maximum frame number is %d",
                        MAX_FRAME_BUFFER);
        response->success = false;
    }
}

void HdRadarNode::SrvSetThrClb(
    const std::shared_ptr<hd_radar_interfaces::srv::SetThr::Request> request,
    const std::shared_ptr<hd_radar_interfaces::srv::SetThr::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "SERVICE: Set thresholds");
    SendThrData(request->sta_threshold, request->sta_azm_sense,
        request->sta_rcs_filter, request->dyn_threshold,
        request->dyn_azm_sense, request->dyn_rcs_filter);
    response->success = true;
}

void HdRadarNode::SrvSetVelClb(
    const std::shared_ptr<hd_radar_interfaces::srv::SetVel::Request> request,
    const std::shared_ptr<hd_radar_interfaces::srv::SetVel::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "SERVICE: Set velocity");
    SendVelData(request->velocity, request->hold_time);
    response->success = true;
}

void HdRadarNode::SrvSetModeClb(
    const std::shared_ptr<hd_radar_interfaces::srv::SetMode::Request> request,
    const std::shared_ptr<hd_radar_interfaces::srv::SetMode::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "SERVICE: Set mode");
    SendModeData(request->mode, request->test_source);
    response->success = true;
}

void HdRadarNode::RequestRawData(uint8_t frames_to_write)
{
    udp_get_raw_t send_packet;

    send_packet.pre_header.version = UDP_PROTOCOL_VER;
    send_packet.pre_header.msg_id  = GET_RAW;
    send_packet.pre_header.length  = sizeof(send_packet);

    RCLCPP_INFO(this->get_logger(), "SERVICE: Requesting %d frames",
                 frames_to_write);
    
    send_packet.raw_args.raw_frame_num = static_cast<uint16_t>(frames_to_write);
    UdpClientSend((char *)&send_packet, sizeof(send_packet));   
}

void HdRadarNode::SendThrData(uint16_t sta_threshold,
     uint16_t sta_azm_sense, int16_t sta_rcs_filter, uint16_t dyn_threshold,
     uint16_t dyn_azm_sense, int16_t dyn_rcs_filter)
{
    udp_msg_arg_thr_t send_packet;

    send_packet.pre_header.version = UDP_PROTOCOL_VER;
    send_packet.pre_header.msg_id  = SET_THR;
    send_packet.pre_header.length  = sizeof(send_packet);
    send_packet.args.sta_threshold = sta_threshold;
    send_packet.args.sta_azm_sense = sta_azm_sense;
    send_packet.args.sta_rcs_filter = sta_rcs_filter;
    send_packet.args.dyn_threshold = dyn_threshold;
    send_packet.args.dyn_azm_sense = dyn_azm_sense;
    send_packet.args.dyn_rcs_filter = dyn_rcs_filter;

    RCLCPP_INFO(this->get_logger(), "SERVICE: Sending thresholds parameters");

    UdpClientSend((char *)&send_packet, sizeof(send_packet));   
}

void HdRadarNode::SendVelData(float velocity, uint32_t hold_time)
{
    udp_msg_arg_vel_t send_packet;

    send_packet.pre_header.version = UDP_PROTOCOL_VER;
    send_packet.pre_header.msg_id  = SET_VEL;
    send_packet.pre_header.length  = sizeof(send_packet);
    send_packet.args.velocity = velocity;
    send_packet.args.hold_time = hold_time;

    RCLCPP_INFO(this->get_logger(), "SERVICE: Sending velocity parameters");
    
    UdpClientSend((char *)&send_packet, sizeof(send_packet));   
}

void HdRadarNode::SendModeData(uint8_t mode, uint8_t test_source)
{
    udp_msg_arg_mode_t send_packet;

    send_packet.pre_header.version = UDP_PROTOCOL_VER;
    send_packet.pre_header.msg_id  = SET_MODE;
    send_packet.pre_header.length  = sizeof(send_packet);
    send_packet.args.mode = mode;
    send_packet.args.test_source = test_source;

    RCLCPP_INFO(this->get_logger(), "SERVICE: Sending mode parameters");
    
    UdpClientSend((char *)&send_packet, sizeof(send_packet));   
}

void HdRadarNode::UdpServerInit()
{
    // Creating socket
    if ((udp_sock_srv_.fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        RCLCPP_ERROR(this->get_logger(), "UDP: Socket creation failed");
        exit(EXIT_FAILURE);
    }
    memset(&udp_sock_srv_.self_addr, 0, sizeof(udp_sock_srv_.self_addr));
    memset(&udp_sock_srv_.dest_addr, 0, sizeof(udp_sock_srv_.dest_addr));

    // Filling server information
    udp_sock_srv_.self_addr.sin_family = AF_INET;
    udp_sock_srv_.self_addr.sin_addr.s_addr = inet_addr(host_ip_.c_str());
    udp_sock_srv_.self_addr.sin_port = htons(receive_port_);

    // Bind socket
    RCLCPP_INFO(this->get_logger(), "UDP: Server socket ip:%s port:%d",
                host_ip_.c_str(), receive_port_);
    if ( bind(udp_sock_srv_.fd, 
        (const struct sockaddr *)&udp_sock_srv_.self_addr,
        sizeof(udp_sock_srv_.self_addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "UDP: Bind failed");
        udp_server_retval_ = -1;
    }

    udp_sock_srv_.mreq.imr_interface.s_addr = inet_addr(host_ip_.c_str());
    udp_sock_srv_.mreq.imr_multiaddr.s_addr = inet_addr(multicast_ip_.c_str());
    if (setsockopt(udp_sock_srv_.fd, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                  (char*) &udp_sock_srv_.mreq, sizeof(udp_sock_srv_.mreq)) < 0){
        RCLCPP_ERROR(this->get_logger(), "UDP: Setsockopt failure");
        udp_server_retval_ = -1;
    }
}

void HdRadarNode::UdpServerStart()
{
    socklen_t len;
    char udp_buf[UDP_BUFFER_LEN];
    len = sizeof(udp_sock_srv_.dest_addr);
    udp_pre_hdr_t pre_header;
    ssize_t buf_len;

    RCLCPP_INFO(this->get_logger(), "UDP: Starting server");
    while (rclcpp::ok()) {
        buf_len = recvfrom(udp_sock_srv_.fd, (char *)udp_buf, UDP_BUFFER_LEN, 
            MSG_WAITALL, ( struct sockaddr *) &udp_sock_srv_.dest_addr, &len);
        if (buf_len < 0) {
            RCLCPP_ERROR(this->get_logger(), "UDP: Buff len ERROR");
            continue;
        }  

        memcpy(&pre_header, udp_buf, sizeof(pre_header));

        // Check protocol version
        if (pre_header.version != UDP_PROTOCOL_VER) {
            RCLCPP_ERROR(this->get_logger(), 
            "UDP: Incompatible protocol version: v %d.%d, expected v %d.%d",
            ((pre_header.version&0xF0) >> 4), (pre_header.version&0x0F),
            ((UDP_PROTOCOL_VER&0xF0) >> 4), (UDP_PROTOCOL_VER&0x0F));
            // continue;
        }

        switch (pre_header.msg_id) {
            case RAW:
                RCLCPP_DEBUG(this->get_logger(),"UDP: Msg type - RAW");
                hd_radar_raw_.ParseRaw((char *)udp_buf, buf_len);
                break;
            case PCL:
                RCLCPP_DEBUG(this->get_logger(),"UDP: Msg type - PCL");
                hd_radar_pcl_.ParsePcl((char *)udp_buf, buf_len);
                break;
            case HEAT:
                RCLCPP_DEBUG(this->get_logger(),"UDP: Msg type - HEAT");
                hd_radar_heat_.ParseHeat((char *)udp_buf, buf_len);
                break;
            
            default:
                RCLCPP_ERROR(this->get_logger(), "UDP: Unknown msg id %d", 
                            pre_header.msg_id);
                exit(EXIT_FAILURE);
                break;
        }
    }
}
void HdRadarNode::UdpClientInit()
{
    // Creating socket
    if ((udp_sock_clnt_.fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        RCLCPP_ERROR(this->get_logger(), "UDP: Socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&udp_sock_clnt_.dest_addr, 0, sizeof(udp_sock_clnt_.dest_addr));

    // Filling client information
    udp_sock_clnt_.dest_addr.sin_family = AF_INET;
    udp_sock_clnt_.dest_addr.sin_addr.s_addr = inet_addr(sensor_ip_.c_str());
    udp_sock_clnt_.dest_addr.sin_port = htons(send_port_);

    RCLCPP_INFO(this->get_logger(), "UDP: Client socket ip:%s port:%d",
                sensor_ip_.c_str(), send_port_);

}

ssize_t HdRadarNode::UdpClientSend(char * msg, size_t len)
{
    ssize_t status;
    status = sendto(udp_sock_clnt_.fd, (const char *)msg, len, 
        MSG_CONFIRM, (const struct sockaddr *) &udp_sock_clnt_.dest_addr,  
            sizeof(udp_sock_clnt_.dest_addr)); 
    if (status < 0) RCLCPP_ERROR(this->get_logger(), "UDP: Send data error");
    else RCLCPP_INFO(this->get_logger(), "UDP: Send data successfully");

    return status;
}

void HdRadarNode::CanServerInit()
{
    // CAN Setup
    RCLCPP_INFO(this->get_logger(), "CAN: Device id: %s",
                can_device_id_.c_str());

    can_sock_srv_.fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(can_sock_srv_.ifr.ifr_name, can_device_id_.c_str());
    ioctl(can_sock_srv_.fd, SIOCGIFINDEX, &can_sock_srv_.ifr);

    can_sock_srv_.addr.can_family = AF_CAN;
    can_sock_srv_.addr.can_ifindex = can_sock_srv_.ifr.ifr_ifindex;
    if (bind(can_sock_srv_.fd , 
        (struct sockaddr *)&can_sock_srv_.addr, sizeof(can_sock_srv_.addr))) {
        RCLCPP_ERROR(this->get_logger(), "CAN: Bind failed");
        can_server_retval_ = -1;
    }
}

void HdRadarNode::CanServerStart() 
{
    can_frame can_buf;
    ssize_t buf_len;

    RCLCPP_INFO(this->get_logger(), "CAN: Starting server");
    while (rclcpp::ok()) {
        buf_len = read(can_sock_srv_.fd, &can_buf, sizeof(can_frame));
        if (buf_len < 0) {
            RCLCPP_ERROR(this->get_logger(), "UDP: Buff len ERROR");
            continue;
        }
        hd_radar_can_.ParsePcl(&can_buf);
    }
}

void HdRadarNode::TimerCallback()
{
    mtx_.lock();
    RCLCPP_INFO(this->get_logger(), 
    "NTP_SYNC %d PCL_STAT %ld PCL_DYN %ld PCL_STAT_C %ld PCL_DYN_C %ld RAW %ld HEAT %ld",
    ntp_sync_, pcl_stat_cnt_, pcl_dyn_cnt_, pcl_stat_c_cnt_, pcl_dyn_c_cnt_, raw_cnt_,
    heat_cnt_);
    mtx_.unlock();
}

HdRadarNode::HdRadarNode() : Node("hd_radar_node")
{
    // Read ros parameters
    ReadParameters();
    RCLCPP_INFO(this->get_logger(), "frame_id_:%s", frame_id_.c_str());
   
    // Init modules
    hd_radar_pcl_.InitParams(&frame_id_, &check_crc16_, &ntp_sync_, &mtx_, this);
    hd_radar_pcl_.BindStatCallback([this]() { HdRadarNode::PublishPclStat(); });
    hd_radar_pcl_.BindDynCallback([this]() { HdRadarNode::PublishPclDyn(); });
    hd_radar_raw_.InitParams(&frame_id_, &check_crc16_, &ntp_sync_, &mtx_, this);
    hd_radar_raw_.BindCallback([this]() { HdRadarNode::PublishRaw(); });
    hd_radar_heat_.InitParams(&frame_id_, &ntp_sync_, &mtx_, this,
                              &img_min_val_, &img_max_val_,
                              &img_pow_, &img_max_rng_);
    hd_radar_heat_.BindCallback([this]() { HdRadarNode::PublishHeat(); });
    hd_radar_can_.InitParams(&frame_id_, this);
    hd_radar_can_.BindStatCallback([this]() { HdRadarNode::PublishPclStatC(); });
    hd_radar_can_.BindDynCallback([this]() { HdRadarNode::PublishPclDynC(); });

    // Create publishers
    rclcpp::SensorDataQoS qos;
    rclcpp::SensorDataQoS qos_raw;
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_raw.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    pub_msg_pcl_stat_  = create_publisher<sensor_msgs::msg::PointCloud2>(
                        topic_ + "/points/static", qos);
    pub_msg_pcl_dyn_ = create_publisher<sensor_msgs::msg::PointCloud2>(
                        topic_ + "/points/dynamic", qos);
    pub_msg_raw_ = create_publisher<hd_radar_interfaces::msg::Raw>(
                        topic_ + "/raw", qos_raw);
    pub_msg_heat_ = create_publisher<hd_radar_interfaces::msg::Heat>(
                        topic_ + "/heat", qos);
    pub_msg_heat_img_ = create_publisher<sensor_msgs::msg::Image>(
                        topic_ + "/heat_img", qos);

    pub_msg_pcl_stat_c_  = create_publisher<sensor_msgs::msg::PointCloud2>(
                                topic_ + "/points/static_c", qos);
    pub_msg_pcl_dyn_c_  = create_publisher<sensor_msgs::msg::PointCloud2>(
                                topic_ + "/points/dynamic_c", qos);

    // Create services
    srv_get_raw_ = this->create_service<hd_radar_interfaces::srv::GetRaw>(
        frame_id_ + "_get_raw", std::bind(&HdRadarNode::SrvGetRawClb, this,
                                std::placeholders::_1, std::placeholders::_2));
    srv_set_thr_ = this->create_service<hd_radar_interfaces::srv::SetThr>(
        frame_id_ + "_set_thr", std::bind(&HdRadarNode::SrvSetThrClb, this,
                                std::placeholders::_1, std::placeholders::_2));
    srv_set_vel_ = this->create_service<hd_radar_interfaces::srv::SetVel>(
        frame_id_ + "_set_vel", std::bind(&HdRadarNode::SrvSetVelClb, this,
                                std::placeholders::_1, std::placeholders::_2));
    srv_set_mode_ = this->create_service<hd_radar_interfaces::srv::SetMode>(
        frame_id_ + "_set_mode", std::bind(&HdRadarNode::SrvSetModeClb, this,
                                std::placeholders::_1, std::placeholders::_2));

    // Udp, Can servers/clients init
    UdpServerInit();
    UdpClientInit();
    CanServerInit();

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                std::bind(&HdRadarNode::TimerCallback, this));
    
    // Start Udp server
    if (udp_server_retval_ == 0){
        std::thread thrd_udp_server(&HdRadarNode::UdpServerStart, this);
        thrd_udp_server.detach();
    }

    // Start Can server
    if (can_server_retval_ == 0){
        std::thread thrd_can_server(&HdRadarNode::CanServerStart, this);
        thrd_can_server.detach();
    }
}

HdRadarNode::~HdRadarNode()
{
    close(udp_sock_srv_.fd);
    close(udp_sock_clnt_.fd);
    close(can_sock_srv_.fd);
};
