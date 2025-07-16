#include "hd_radar_driver/modules/hd_radar_heat.hpp"

void HdRadarHeat::BindCallback(std::function<void()> func)
{
    PubCallback = std::bind(func);
}

void HdRadarHeat::ParseHeat(char * buffer, size_t buf_len)
{

    memset(&msg_heat_, 0, sizeof(udp_msg_heat_t));
    memcpy(&msg_heat_, buffer, static_cast<size_t>(buf_len));
    // memset(&buffer_, 0, sizeof(msg_heat_t));

    if (heat_full_.size() != msg_heat_.heat_header.udp_total*UDP_PAYLOAD_SIZE) {
        heat_full_.resize(msg_heat_.heat_header.udp_total*UDP_PAYLOAD_SIZE);
        std::fill(heat_full_.begin(), heat_full_.end(), 0);
    }
    // RCLCPP_INFO(node_->get_logger(), "data %d ", buffer[0]);
    RCLCPP_DEBUG(node_->get_logger(), "HEAT: frame_current %d curr_frame %ld",
    msg_heat_.heat_header.frame_cnt, heat_cur_frame_);
    RCLCPP_DEBUG(node_->get_logger(), "HEAT: idx %d total %d",
    msg_heat_.heat_header.udp_idx, msg_heat_.heat_header.udp_total);
    RCLCPP_DEBUG(node_->get_logger(), "HEAT: map_h_size %d map_v_size %d",
    msg_heat_.heat_header.map_h_size, msg_heat_.heat_header.map_v_size);

    if (msg_heat_.heat_header.frame_cnt != heat_cur_frame_) {
        if (heat_cur_subframe_ != 0)
        // RCLCPP_INFO(node_->get_logger(), "Pub 1");
        PublishHeat();
        heat_cur_subframe_ = 0;
        time_stamp_heat_ = node_->now();
        heat_cur_frame_ = msg_heat_.heat_header.frame_cnt;
    }

    memcpy(&heat_full_[msg_heat_.heat_header.udp_idx*
        UDP_PAYLOAD_SIZE/sizeof(float)], &msg_heat_.data[0], UDP_PAYLOAD_SIZE);
            heat_cur_subframe_++;

    if (msg_heat_.heat_header.udp_idx == msg_heat_.heat_header.udp_total) {
        // RCLCPP_INFO(node_->get_logger(), "Pub 2");
        PublishHeat();
        heat_cur_subframe_ = 0;
    }
}

void HdRadarHeat::FillHeat()
{
    //Filling native heat map message
    msg_heat_hd_.h_size = msg_heat_.heat_header.map_h_size;
    msg_heat_hd_.v_size = msg_heat_.heat_header.map_v_size;
    msg_heat_hd_.array_size = heat_full_.size();
    msg_heat_hd_.heat_data = heat_full_;

    //Filling heat map image message
    cv::Mat refined_heat = cv::Mat(msg_heat_.heat_header.map_h_size, 
        msg_heat_.heat_header.map_v_size, CV_8UC1, 0.0);
    cv::Mat refined_XY_255_cv_colour_;

    float max = 0;
    float min = FLT_MAX;

    for (size_t i = 0; i < msg_heat_.heat_header.map_h_size; ++i) {
        for (size_t j = 0; j < msg_heat_.heat_header.map_v_size; ++j) {
        float float_val = heat_full_[i*msg_heat_.heat_header.map_h_size + j];
        float q = (img_max_val_ - img_min_val_)/256.0;
        uint32_t pixel = float_val/q;
        refined_heat.at<uint8_t>(i, j) = pixel < 255 ? pixel : 255;

        if (float_val > max) max = float_val;
        if (float_val < min) min = float_val;
        }
    }

    int cx = refined_heat.cols;
    int cy = refined_heat.rows/2;

    cv::Mat tmp;
    cv::Mat q0(refined_heat, cv::Rect(0, 0, cx, cy));
    cv::Mat q1(refined_heat, cv::Rect(0, cy, cx, cy));

    q0.copyTo(tmp);
    q1.copyTo(q0);
    tmp.copyTo(q1);

    applyColorMap(refined_heat, refined_XY_255_cv_colour_, cv::COLORMAP_JET);

    std_msgs::msg::Header header_;

    // RCLCPP_INFO(node_->get_logger(), "frame_id: %s", frame_id.c_str());
    RCLCPP_DEBUG(node_->get_logger(), "HEAT: img_min_val_: %f, img_max_val_: %f",
                img_min_val_, img_max_val_);
    
    header_.frame_id = frame_id_;
    header_.stamp = node_->now();

    msg_img_ = cv_bridge::CvImage(header_, "bgr8", 
                                refined_XY_255_cv_colour_).toImageMsg();
}

void HdRadarHeat::PublishHeat()
{
    FillHeat();
    PubCallback();
}

void HdRadarHeat::InitParams(std::string * frame_id,  rclcpp::Node * node, 
    double * img_min_val, double * img_max_val) 
{
        frame_id_ = *frame_id;
        node_ = node;
        img_min_val_ = *img_min_val;
        img_max_val_ = *img_max_val;
}

HdRadarHeat::HdRadarHeat()
{
}

HdRadarHeat::~HdRadarHeat()
{
}
