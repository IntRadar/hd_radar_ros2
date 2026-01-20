#ifndef HD_RADAR_CAN_HPP
#define HD_RADAR_CAN_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "hd_radar_driver/hd_radar_structs.hpp"
#include "crc16/crc16_ccitt.hpp"

class HdRadarCan 
{
 public:
  std::string frame_id_;

  // Messages
  sensor_msgs::msg::PointCloud2 msg_pcl2_;

  // Pointer to hd_driver_node
  rclcpp::Node *node_;

  void InitParams(std::string * frame_id,  rclcpp::Node * node);

  //Bind callback function
  void BindStatCallback(std::function<void()> func);
  void BindDynCallback(std::function<void()> func);

  void ParsePcl(can_frame* can_buffer);

  HdRadarCan();
  ~HdRadarCan();

 private:
  // ROS2
  rclcpp::Time time_stamp_;
  
  can_msg_pcl_t msg_can_pcl_{};
  int32_t can_data_remains_ = 0;
  uint32_t can_data_offset_ = 0U;

  void PublishPcl();
  void FillPcl();

  std::function<void()> PubStatCallback;
  std::function<void()> PubDynCallback;
};

#endif  // HD_RADAR_CAN_HPP