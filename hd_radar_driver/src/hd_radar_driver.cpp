
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

    this->declare_parameter("min_val", 0.0);
    img_min_val_ = this->get_parameter("min_val").as_double();

    this->declare_parameter("max_val", 1.0e4);
    img_max_val_ = this->get_parameter("max_val").as_double();

    this->declare_parameter("check_crc16", false);
    check_crc16_ = this->get_parameter("check_crc16").as_bool();
}

void HdRadarNode::PublishPclStat() 
{
    RCLCPP_DEBUG(this->get_logger(), "Callback pcl stat");
    pub_msg_pcl_stat_->publish(hd_radar_pcl_.msg_pcl2_);
}

void HdRadarNode::PublishPclDyn() 
{
    RCLCPP_DEBUG(this->get_logger(), "Callback pcl dyn");
    pub_msg_pcl_dyn_->publish(hd_radar_pcl_.msg_pcl2_);
}

void HdRadarNode::PublishRaw() 
{
    RCLCPP_DEBUG(this->get_logger(), "Callback raw ");
    // hd_radar_raw_.msg_raw_hd_.header.frame_id = frame_id_;
    pub_msg_raw_->publish(hd_radar_raw_.msg_raw_hd_);
}

void HdRadarNode::PublishHeat()
{
    RCLCPP_INFO(this->get_logger(), "Callback heat ");

    pub_msg_heat_->publish(hd_radar_heat_.msg_heat_hd_);
    pub_msg_heat_img_->publish(*hd_radar_heat_.msg_img_);
}

void HdRadarNode::SrvGetRawClb(
    const std::shared_ptr<hd_radar_interfaces::srv::GetRaw::Request> request,
    const std::shared_ptr<hd_radar_interfaces::srv::GetRaw::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Service get raw");
    if (request->frames_num <= MAX_FRAME_BUFFER) {
        response->success = true;
        RequestRawData(request->frames_num);
        } else if (request->frames_num == 0) {
        RCLCPP_ERROR(this->get_logger(), "Minimum frame number is 1");
        response->success = false;
        }
        else {
        RCLCPP_ERROR(this->get_logger(), "Maximum frame number is %d",
                        MAX_FRAME_BUFFER);
        response->success = false;
        }
}

void HdRadarNode::SrvSetThrClb(
    const std::shared_ptr<hd_radar_interfaces::srv::SetThr::Request> request,
    const std::shared_ptr<hd_radar_interfaces::srv::SetThr::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Service set thresholds");
    SendThrData(request->sta_threshold, request->sta_azm_sense,
        request->sta_rcs_filter, request->dyn_threshold,
        request->dyn_azm_sense, request->dyn_rcs_filter);
    response->success = true;
}

void HdRadarNode::SrvSetVelClb(
    const std::shared_ptr<hd_radar_interfaces::srv::SetVel::Request> request,
    const std::shared_ptr<hd_radar_interfaces::srv::SetVel::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Service set velocity");
    SendVelData(request->velocity, request->hold_time);
    response->success = true;
}

void HdRadarNode::SrvSetModeClb(
    const std::shared_ptr<hd_radar_interfaces::srv::SetMode::Request> request,
    const std::shared_ptr<hd_radar_interfaces::srv::SetMode::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Service set mode");
    SendModeData(request->mode, request->test_source);
    response->success = true;
}

void HdRadarNode::RequestRawData(uint8_t frames_to_write)
{
    get_raw_t send_packet;

    send_packet.pre_header.version = PROTOCOL_VER;
    send_packet.pre_header.msg_id  = GET_RAW;
    send_packet.pre_header.length  = sizeof(send_packet);

    RCLCPP_INFO(this->get_logger(), "Requesting %d frames", frames_to_write);
    
    send_packet.raw_args.raw_frame_num = static_cast<uint16_t>(frames_to_write);
    UdpClientSend((char *)&send_packet, sizeof(send_packet));   
}

void HdRadarNode::SendThrData(uint16_t sta_threshold,
     uint16_t sta_azm_sense, int16_t sta_rcs_filter, uint16_t dyn_threshold,
     uint16_t dyn_azm_sense, int16_t dyn_rcs_filter)
{
    msg_arg_thr_t send_packet;

    send_packet.pre_header.version = PROTOCOL_VER;
    send_packet.pre_header.msg_id  = SET_THR;
    send_packet.pre_header.length  = sizeof(send_packet);
    send_packet.args.sta_threshold = sta_threshold;
    send_packet.args.sta_azm_sense = sta_azm_sense;
    send_packet.args.sta_rcs_filter = sta_rcs_filter;
    send_packet.args.dyn_threshold = dyn_threshold;
    send_packet.args.dyn_azm_sense = dyn_azm_sense;
    send_packet.args.dyn_rcs_filter = dyn_rcs_filter;

    RCLCPP_INFO(this->get_logger(), "Sending thresholds parameters");

    UdpClientSend((char *)&send_packet, sizeof(send_packet));   
}

void HdRadarNode::SendVelData(float velocity, uint32_t hold_time)
{
    msg_arg_vel_t send_packet;

    send_packet.pre_header.version = PROTOCOL_VER;
    send_packet.pre_header.msg_id  = SET_VEL;
    send_packet.pre_header.length  = sizeof(send_packet);
    send_packet.args.velocity = velocity;
    send_packet.args.hold_time = hold_time;

    RCLCPP_INFO(this->get_logger(), "Sending velocity parameters");
    
    UdpClientSend((char *)&send_packet, sizeof(send_packet));   
}

void HdRadarNode::SendModeData(uint8_t mode, uint8_t test_source)
{
    msg_arg_mode_t send_packet;

    send_packet.pre_header.version = PROTOCOL_VER;
    send_packet.pre_header.msg_id  = SET_MODE;
    send_packet.pre_header.length  = sizeof(send_packet);
    send_packet.args.mode = mode;
    send_packet.args.test_source = test_source;

    RCLCPP_INFO(this->get_logger(), "Sending mode parameters");
    
    UdpClientSend((char *)&send_packet, sizeof(send_packet));   
}

void HdRadarNode::UdpServerInit()
{
    // Creating socket
    if ((sock_server_.fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        RCLCPP_INFO(this->get_logger(), "Socket creation failed");
        exit(EXIT_FAILURE);
    }
    memset(&sock_server_.self_addr, 0, sizeof(sock_server_.self_addr));
    memset(&sock_server_.dest_addr, 0, sizeof(sock_server_.dest_addr));

    // Filling server information
    sock_server_.self_addr.sin_family = AF_INET;
    sock_server_.self_addr.sin_addr.s_addr = inet_addr(host_ip_.c_str());
    sock_server_.self_addr.sin_port = htons(receive_port_);

    // u_int yes = 1;
    // if (setsockopt(sock_server_.fd, SOL_SOCKET, SO_REUSEADDR,
    //     (char*) &yes, sizeof(yes)) < 0){
    //     RCLCPP_ERROR(this->get_logger(), "Reusing ADDR failed");
    //   }

    // Bind socket
    RCLCPP_INFO(this->get_logger(), "Bind server socket ip:%s port:%d", host_ip_.c_str(), PORT_RECEIVE);
    if ( bind(sock_server_.fd, (const struct sockaddr *)&sock_server_.self_addr,
         sizeof(sock_server_.self_addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Bind failed");
        exit(EXIT_FAILURE);
    }

    
    sock_server_.mreq.imr_interface.s_addr = inet_addr(host_ip_.c_str());
    sock_server_.mreq.imr_multiaddr.s_addr = inet_addr(multicast_ip_.c_str());
    if (setsockopt(sock_server_.fd, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                  (char*) &sock_server_.mreq, sizeof(sock_server_.mreq)) < 0){
        RCLCPP_ERROR(this->get_logger(), "Setsockopt failure");
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
}
void HdRadarNode::UdpServerStart()
{
    socklen_t len;

    len = sizeof(sock_server_.dest_addr);
    pre_hdr_t pre_header;

    RCLCPP_INFO(this->get_logger(), "Starting server");
    do {
        buf_len_ = recvfrom(sock_server_.fd, (char *)buffer_, BUFFER_LEN, 
            MSG_WAITALL, ( struct sockaddr *) &sock_server_.dest_addr, &len);
        memcpy(&pre_header, buffer_, sizeof(pre_header));

        // Check protocol version
        if (pre_header.version != PROTOCOL_VER)
        {
            RCLCPP_ERROR(this->get_logger(), "Incompatible protocol version");
            exit(EXIT_FAILURE);
        }

        switch (pre_header.msg_id)
        {
            case RAW:
                RCLCPP_INFO(this->get_logger(),"Message type: RAW");
                hd_radar_raw_.ParseRaw((char *)buffer_, buf_len_);
                break;
            case PCL:
                RCLCPP_INFO(this->get_logger(),"Message type: PCL");
                hd_radar_pcl_.ParsePcl((char *)buffer_, buf_len_);
                break;
            case HEAT:
                RCLCPP_INFO(this->get_logger(),"Message type: HEAT");
                hd_radar_heat_.ParseHeat((char *)buffer_, buf_len_);
                break;
            
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown message id %d", 
                            pre_header.msg_id);
                exit(EXIT_FAILURE);
                break;
        }
    }
    while (buf_len_ > 0);
}
void HdRadarNode::UdpClientInit()
{
    // Creating socket
    if ((sock_client_.fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        RCLCPP_INFO(this->get_logger(), "Socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&sock_client_.dest_addr, 0, sizeof(sock_client_.dest_addr));

    // Filling client information
    sock_client_.dest_addr.sin_family = AF_INET;
    sock_client_.dest_addr.sin_addr.s_addr = inet_addr(sensor_ip_.c_str());
    sock_client_.dest_addr.sin_port = htons(send_port_);

}

ssize_t HdRadarNode::UdpClientSend(char * msg, size_t len)
{
    ssize_t status;
    status = sendto(sock_client_.fd, (const char *)msg, len, 
        MSG_CONFIRM, (const struct sockaddr *) &sock_client_.dest_addr,  
            sizeof(sock_client_.dest_addr)); 
    if (status < 0) RCLCPP_INFO(this->get_logger(), "Send error");
    else RCLCPP_INFO(this->get_logger(), "Send successfully");

    return status;
}

HdRadarNode::HdRadarNode() : Node("hd_radar_node")
{
    // Read ros parameters
    ReadParameters();
    RCLCPP_INFO(this->get_logger(), "frame_id_:%s", frame_id_.c_str());
    // Init modules
    hd_radar_pcl_.InitParams(&frame_id_, &check_crc16_, this);
    hd_radar_pcl_.BindStatCallback([this]() { HdRadarNode::PublishPclStat(); });
    hd_radar_pcl_.BindDynCallback([this]() { HdRadarNode::PublishPclDyn(); });
    hd_radar_raw_.InitParams(&frame_id_, &check_crc16_, this);
    hd_radar_raw_.BindCallback([this]() { HdRadarNode::PublishRaw(); });
    hd_radar_heat_.InitParams(&frame_id_, this, &img_min_val_, &img_max_val_);
    hd_radar_heat_.BindCallback([this]() { HdRadarNode::PublishHeat(); });

    // Create publishers
    rclcpp::SensorDataQoS qos;

    pub_msg_pcl_stat_  = create_publisher<sensor_msgs::msg::PointCloud2>(
                        topic_ + "/points/static", qos);
    pub_msg_pcl_dyn_ = create_publisher<sensor_msgs::msg::PointCloud2>(
                        topic_ + "/points/dynamic", qos);
    pub_msg_raw_ = create_publisher<hd_radar_interfaces::msg::Raw>(
                        topic_ + "/raw", qos);
    pub_msg_heat_ = create_publisher<hd_radar_interfaces::msg::Heat>(
                        topic_ + "/heat", qos);
    pub_msg_heat_img_ = create_publisher<sensor_msgs::msg::Image>(
                        topic_ + "/heat_img", qos);

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
    // Udp server/client init
    UdpServerInit();
    UdpClientInit();

    // Start Udp server
    std::thread thrd_udp_server(&HdRadarNode::UdpServerStart, this);
    thrd_udp_server.detach(); 
}

HdRadarNode::~HdRadarNode(){};
