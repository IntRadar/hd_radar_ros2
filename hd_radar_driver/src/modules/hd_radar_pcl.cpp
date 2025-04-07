#include "hd_radar_driver/modules/hd_radar_pcl.hpp"

void HdRadarPcl::BindStatCallback(std::function<void()> func)
{
    PubStatCallback = std::bind(func);
}
void HdRadarPcl::BindDynCallback(std::function<void()> func)
{
    PubDynCallback = std::bind(func);
}
void HdRadarPcl::ParsePcl(char * buffer, size_t buf_len)
{
    RCLCPP_DEBUG(node_->get_logger(),"UDP: Parsing PCL message");
    memset(&msg_pcl_, 0, sizeof(udp_msg_pcl_t));
    memcpy(&msg_pcl_, buffer, buf_len);

    uint16_t crc16 = CRC16Get(reinterpret_cast<void*>(msg_pcl_.payload),
                    msg_pcl_.pcl_header.udp_pnts_num*sizeof(udp_pcl_data_t));
    RCLCPP_DEBUG(node_->get_logger(), "crc16: %d, crc16 from radar: %d",
                crc16, msg_pcl_.pcl_header.crc16);

    if (check_crc16_ && crc16 != msg_pcl_.pcl_header.crc16) {
        RCLCPP_ERROR(node_->get_logger(), "Wrong crc16");
        exit(EXIT_FAILURE);
    }

    if (msg_pcl_.pcl_header.frame_cnt != pcl_cur_frame_ ||
        msg_pcl_.pcl_header.pcl_type != pcl_cur_cloud_) {
        if (pcl_pnt_received_ != 0) {
        PublishPcl();
        }
        time_stamp_pcl_ = node_->now();
        pcl_pnt_received_ = 0;
        pcl_pnts_in_frame_.clear();
        pcl_cur_frame_ = msg_pcl_.pcl_header.frame_cnt;
        pcl_cur_cloud_ = msg_pcl_.pcl_header.pcl_type;
    }

    for (size_t i = 0; i < msg_pcl_.pcl_header.udp_pnts_num; ++i) {
        pcl_pnts_in_frame_.push_back(msg_pcl_.payload[i]);
    }
    pcl_pnt_received_ += msg_pcl_.pcl_header.udp_pnts_num;
    if (pcl_pnts_in_frame_.size() > pcl_pnt_received_) {
        pcl_pnts_in_frame_.resize(pcl_pnt_received_);
    }

    if (msg_pcl_.pcl_header.udp_idx == msg_pcl_.pcl_header.udp_total) {
        PublishPcl();
        pcl_pnt_received_ = 0;
        pcl_pnts_in_frame_.clear();
    }
}

void HdRadarPcl::PublishPcl() 
{
    if (enable_ntp_) {
        
        time_stamp_radar_ = (msg_pcl_.pcl_header.tv_usec_lsb << 8) | 
                            (msg_pcl_.pcl_header.tv_usec_msb);
        time_stamp_pcl_ = rclcpp::Time(time_stamp_radar_ *
                                      static_cast<uint64_t>(1e3));
        }

    // PointCloud fields initialization
    FillPcl();

    // Publish message   
    if (pcl_cur_cloud_ == 0) {
        PubStatCallback();
    } else {
        PubDynCallback();
    }
    
    // RCLCPP_INFO(node_->get_logger(),"Targets were founded: %d Type: %d",
    //             pcl_pnt_received_, pcl_cur_cloud_);
}

void HdRadarPcl::FillPcl() 
{
    //Msg header
    msg_pcl2_.header = std_msgs::msg::Header();
    msg_pcl2_.header.stamp = time_stamp_pcl_;
    msg_pcl2_.header.frame_id = frame_id_;

    msg_pcl2_.height = 1;
    msg_pcl2_.width = pcl_pnts_in_frame_.size();

    // RCLCPP_INFO(this->get_logger(), "msg_pcl2_.width: %d", msg_pcl2_.width );

    msg_pcl2_.is_dense = true;

    //Total number of bytes per point
    msg_pcl2_.point_step = 28;
    msg_pcl2_.row_step = msg_pcl2_.point_step * msg_pcl2_.width * 
                        msg_pcl2_.height;
    msg_pcl2_.data.resize(msg_pcl2_.row_step);

    //Iterators for PointCloud msg
    sensor_msgs::PointCloud2Iterator<float> iterX(msg_pcl2_, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(msg_pcl2_, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(msg_pcl2_, "z");
    sensor_msgs::PointCloud2Iterator<float> iterV(msg_pcl2_, "v");
    sensor_msgs::PointCloud2Iterator<float> iterSelfV(msg_pcl2_, "self_v");
    sensor_msgs::PointCloud2Iterator<float> iterSnr(msg_pcl2_, "snr");
    sensor_msgs::PointCloud2Iterator<float> iterRcs(msg_pcl2_, "rcs");

    double x, y, z, v, self_v;

    for (auto point : pcl_pnts_in_frame_) {

        x = point.range * cos(point.azm) * cos(point.elv);
        y = -point.range * sin(point.azm) * cos(point.elv);
        z = point.range * sin(point.elv);
        v = point.velocity;
        self_v = msg_pcl_.pcl_header.self_velocity;

        // RCLCPP_INFO(node_->get_logger(), "X: %f Y: %f Z: %f V: %f", x, y, z, v);

        *iterX = static_cast<float>(std::round(x*1000) / 1000);
        *iterY = static_cast<float>(std::round(y*1000) / 1000);
        *iterZ = static_cast<float>(std::round(z*1000) / 1000);
        *iterV = static_cast<float>(std::round(v*100) / 100);
        *iterSelfV = static_cast<float>(std::round(self_v*100) / 100);
        *iterSnr = static_cast<float>(std::round(point.snr*10) / 10);
        *iterRcs = static_cast<float>(std::round(point.rcs*10) / 10);

        ++iterX;
        ++iterY;
        ++iterZ;
        ++iterV;
        ++iterSelfV;
        ++iterSnr;
        ++iterRcs;
        }
}

void HdRadarPcl::InitParams(std::string * frame_id, bool * check_crc16,
                            rclcpp::Node * node) 
{
        frame_id_ = *frame_id;
        check_crc16_ = *check_crc16;
        node_ = node;
}

HdRadarPcl::HdRadarPcl()
{
    //Modifer to describe what the fields are
    sensor_msgs::PointCloud2Modifier pcl_modifier(msg_pcl2_);
    pcl_modifier.setPointCloud2Fields(7, 
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "v", 1, sensor_msgs::msg::PointField::FLOAT32,
    "self_v", 1, sensor_msgs::msg::PointField::FLOAT32, 
    "snr", 1, sensor_msgs::msg::PointField::FLOAT32, 
    "rcs", 1, sensor_msgs::msg::PointField::FLOAT32);
}

HdRadarPcl::~HdRadarPcl()
{
}
