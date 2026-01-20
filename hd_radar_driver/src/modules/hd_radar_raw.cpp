#include "hd_radar_driver/modules/hd_radar_raw.hpp"

void HdRadarRaw::BindCallback(std::function<void()> func)
{
    PubCallback = std::bind(func);
}

void HdRadarRaw::ParseRaw(char * buffer, size_t buf_len) 
{
    RCLCPP_DEBUG(node_->get_logger(),"RAW: Parsing RAW message");

    size_t udp_total_reserved = 0;

    memset(&msg_raw_, 0, sizeof(udp_msg_raw_t));
    memcpy(&msg_raw_, buffer, static_cast<size_t>(buf_len));
    // memset(&buffer_, 0, sizeof(msg_raw_t));

    uint16_t crc16 = CRC16Get(reinterpret_cast<void*>(msg_raw_.data),
                             sizeof(msg_raw_.data));
  
    RCLCPP_DEBUG(node_->get_logger(), "RAW: crc16: %d, crc16 from radar: %d",
                 crc16, msg_raw_.raw_header.crc16);

    if (check_crc16_ && crc16 != msg_raw_.raw_header.crc16) {
        RCLCPP_ERROR(node_->get_logger(), "RAW: Wrong crc16");
        exit(EXIT_FAILURE);
    }

    if (udp_total_reserved < msg_raw_.raw_header.udp_total) {
        udp_total_reserved = msg_raw_.raw_header.udp_total;
        msg_raw_hd_.raw_data.resize(udp_total_reserved * UDP_PAYLOAD_SIZE);
    }

    if (msg_raw_.raw_header.udp_idx == 0) {
        msg_raw_hd_.header.frame_id = frame_id_;

        // NTP sync check
        uint64_t high_bytes, time_stamp_radar;
        high_bytes = static_cast<uint64_t>(msg_raw_.raw_header.tv_usec_msb);
        time_stamp_radar = (high_bytes << 32 |
                        static_cast<uint64_t>(msg_raw_.raw_header.tv_usec_lsb));
        uint64_t delta_time = ((uint64_t)(node_->now().nanoseconds()/1000) -
                                time_stamp_radar);
        RCLCPP_INFO(node_->get_logger(), "RAW: delta: %ld", delta_time);

        // Check if delta_time greater then 1 sec
        if (delta_time > 1000000) {
            msg_raw_hd_.header.stamp = node_->now();
            mtx_->lock();
            *ntp_sync_ = false;
            mtx_->unlock();
        }
        else {
            msg_raw_hd_.header.stamp = rclcpp::Time(time_stamp_radar * 
                                            static_cast<uint64_t>(1e3));
            mtx_->lock();
            *ntp_sync_ = true;
            mtx_->unlock();
        }

        raw_cur_subframe_ = 0;
        raw_cur_frame_ = msg_raw_.raw_header.frame_cnt;
        // memset(&msg_raw_hd_.raw_data[0], 0, udp_total_reserved * UDP_PAYLOAD_SIZE);
        RCLCPP_INFO(node_->get_logger(), "RAW: Receiving raw msg %ld", raw_cur_frame_);
    }
    raw_cur_subframe_++;
   
    if (raw_cur_frame_ == msg_raw_.raw_header.frame_cnt) {
        memcpy(&msg_raw_hd_.raw_data[UDP_PAYLOAD_SIZE *
            msg_raw_.raw_header.udp_idx], msg_raw_.data, UDP_PAYLOAD_SIZE);
    }

    if (raw_cur_subframe_ == msg_raw_.raw_header.udp_total) {
        PublishRaw();
    }
}

void HdRadarRaw::PublishRaw() {

    PubCallback();
}

void HdRadarRaw::InitParams(std::string * frame_id, bool * check_crc16,
                            bool * ntp_sync, std::mutex * mtx,
                            rclcpp::Node * node) 
{
    frame_id_ = *frame_id;
    check_crc16_ = *check_crc16;
    ntp_sync_ = ntp_sync;
    mtx_ = mtx;
    node_ = node;
}

HdRadarRaw::HdRadarRaw()
{
}

HdRadarRaw::~HdRadarRaw()
{
}
