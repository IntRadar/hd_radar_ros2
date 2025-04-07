#include "hd_radar_driver/modules/hd_radar_raw.hpp"

void HdRadarRaw::BindCallback(std::function<void()> func)
{
    PubCallback = std::bind(func);
}

void HdRadarRaw::ParseRaw(char * buffer, size_t buf_len) 
{
    RCLCPP_DEBUG(node_->get_logger(),"Parsing RAW message");

    size_t udp_total_reserved = 0;

    memset(&msg_raw_, 0, sizeof(udp_msg_raw_t));
    memcpy(&msg_raw_, buffer, static_cast<size_t>(buf_len));
    // memset(&buffer_, 0, sizeof(msg_raw_t));

    uint16_t crc16 = CRC16Get(reinterpret_cast<void*>(msg_raw_.data),
                             sizeof(msg_raw_.data));
  
    RCLCPP_DEBUG(node_->get_logger(), "crc16: %d, crc16 from radar: %d",
                 crc16, msg_raw_.raw_header.crc16);

    if (check_crc16_ && crc16 != msg_raw_.raw_header.crc16) {
        RCLCPP_ERROR(node_->get_logger(), "Wrong crc16");
        exit(EXIT_FAILURE);
    }

    time_stamp_raw_ = node_->now();

    if (udp_total_reserved < msg_raw_.raw_header.udp_total) {
        udp_total_reserved = msg_raw_.raw_header.udp_total;
        msg_raw_hd_.raw_data.resize(udp_total_reserved * UDP_PAYLOAD_SIZE);
    }

    if (msg_raw_.raw_header.udp_idx == 0) {
        time_stamp_raw_ = node_->now();
        raw_cur_subframe_ = 0;
        raw_cur_frame_ = msg_raw_.raw_header.frame_cnt;
        for (unsigned int i = 0; i < udp_total_reserved * UDP_PAYLOAD_SIZE; i++)
        msg_raw_hd_.raw_data[i] = 0;
        RCLCPP_INFO(node_->get_logger(), "reset");
    }
    raw_cur_subframe_++;
    RCLCPP_INFO(node_->get_logger(), "dg=%d nf=%d count=%ld",
                msg_raw_.raw_header.udp_idx,
                msg_raw_.raw_header.frame_cnt,
                raw_cur_subframe_);

    if (raw_cur_frame_ == msg_raw_.raw_header.frame_cnt) {
        memcpy(&msg_raw_hd_.raw_data[UDP_PAYLOAD_SIZE *
                msg_raw_.raw_header.udp_idx], msg_raw_.data, UDP_PAYLOAD_SIZE);
    }

    if (raw_cur_subframe_ == msg_raw_.raw_header.udp_total) {
        PublishRaw();
    }
}

void HdRadarRaw::PublishRaw() {
    // RCLCPP_INFO(this->get_logger(), "Publish Raw msg");
    msg_raw_hd_.header.stamp = time_stamp_raw_;
    msg_raw_hd_.header.frame_id = frame_id_;
    PubCallback();
}

void HdRadarRaw::InitParams(std::string * frame_id, bool * check_crc16,
                            rclcpp::Node * node) 
{
    frame_id_ = *frame_id;
    check_crc16_ = *check_crc16;
    node_ = node;
}

HdRadarRaw::HdRadarRaw()
{
}

HdRadarRaw::~HdRadarRaw()
{
}
