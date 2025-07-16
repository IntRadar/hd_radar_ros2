#include "hd_radar_driver/modules/hd_radar_can.hpp"

void HdRadarCan::BindStatCallback(std::function<void()> func)
{
    PubStatCallback = std::bind(func);
}

void HdRadarCan::BindDynCallback(std::function<void()> func)
{
    PubDynCallback = std::bind(func);
}

void HdRadarCan::PublishPcl() 
{
    // PointCloud fields initialization
    FillPcl();

    // Publish message   
  if (msg_can_pcl_.header.flags == 0) {
    RCLCPP_DEBUG(node_->get_logger(), 
                "CAN: Publishing static cloud with %u points",
                msg_can_pcl_.header.pnts_num);
    PubStatCallback();
  } else {
    RCLCPP_DEBUG(node_->get_logger(), 
                  "CAN: Publishing dynamic cloud with %u points",
                    msg_can_pcl_.header.pnts_num);
      PubDynCallback();
  }
}

void HdRadarCan::FillPcl() 
{
  msg_pcl2_.header = std_msgs::msg::Header();
  msg_pcl2_.header.frame_id = frame_id_;
  msg_pcl2_.header.stamp = node_->now();

  msg_pcl2_.height = 1;
  msg_pcl2_.width = msg_can_pcl_.header.pnts_num;  
  
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

  can_pcl_data_t point;

  for (int i = 0; i < int(msg_can_pcl_.header.pnts_num); ++i) {

    double x, y, z, self_v, v, azm, elv, range;

    point = msg_can_pcl_.point[i];

    azm = point.azimuth * 0.001;
    elv = point.elevation * 0.001;
    range = point.range * 0.01;
    v = point.velocity * 0.01;

    x = range * cos(azm) * cos(elv);
    y = -range * sin(azm) * cos(elv);
    z = range * sin(elv);
    self_v = msg_can_pcl_.header.ego_velocity * 0.01;

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

void HdRadarCan::ParsePcl(can_frame* can_buffer)
{
  /* We treat CAN_PREHEADER_MSG_ID_OFFSET as CAN device identifier.
   * This parameter should be modified to configurable variable to support 
   * of multiple devices on the same CAN bus.
   * Thre main check is performed to collect data from a specific device.
   */
  if ((can_buffer->can_id & 
      CAN_PREHEADER_MSG_ID_OFFSET) == CAN_PREHEADER_MSG_ID_OFFSET) {
    /* Wait for pre-header id to find the beginning of datagram */
    if (can_buffer->can_id == CAN_PREHEADER_MSG_ID_OFFSET) {

      std::stringstream ss;
      can_pre_hdr_t* pre_hdr = (can_pre_hdr_t*)can_buffer->data;
  
      /* Here we check for any protocol mismatch */
      if (pre_hdr->version != CAN_PROTOCOL_VER) {
        RCLCPP_INFO(node_->get_logger(), 
            "CAN: Incompatible protocol version: v %d.%d, expected v %d.%d",
            ((pre_hdr->version&0xF0) >> 4), (pre_hdr->version&0x0F), 
            ((CAN_PROTOCOL_VER&0xF0) >> 4) , (CAN_PROTOCOL_VER&0x0F));
        return;
      }
      else if (pre_hdr->msg_id != CAN_PCL_MSG_ID) {
        RCLCPP_INFO(node_->get_logger(),
           "CAN: Wrong Can Msg ID: 0x%.2X, need 0x%.2X",
            pre_hdr->msg_id, CAN_PCL_MSG_ID);
        return;
      }

      /* Here we check for data length mismatch */ 
      if (can_data_remains_ > 0) {
        RCLCPP_INFO(node_->get_logger(),
        "CAN: Data inconsistency - new data before publish, data remains = %d",
        can_data_remains_);

        PublishPcl();
      }
      else if (can_data_remains_ < 0) {
        RCLCPP_INFO(node_->get_logger(),
        "CAN: Data inconsistency - new data before publish, data remains = %d",
        -can_data_remains_);
        PublishPcl();
      }
      /* In any case reset offset and set data reminder length for new data */
      can_data_remains_ = pre_hdr->length;
      can_data_offset_ = 0U;
    }
    
    /* Positive reminder indicate that pre-header has been received */
    if (can_data_remains_ > 0) {

      memcpy((uint8_t*)&msg_can_pcl_ + can_data_offset_,
            can_buffer->data, can_buffer->can_dlc);
   
      can_data_offset_ += can_buffer->can_dlc;
      can_data_remains_ -= can_buffer->can_dlc;

      if (can_data_remains_ == 0) {
        /* Succsessful capture of entire CAN datagram */
        PublishPcl();
      }
    }
  }
}

void HdRadarCan::InitParams(std::string * frame_id,  rclcpp::Node * node) 
{
  frame_id_ = *frame_id;
  node_ = node;
}

HdRadarCan::HdRadarCan()
{
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

HdRadarCan::~HdRadarCan() 
{
}
