//
// Created by ch on 24-11-23.
//
#pragma once

#include <array>
#include <cstdint>
#include <ros/ros.h>

#include "rm_vt/common/data.h"

namespace rm_vt
{
class VideoTransmission
{
public:
  explicit VideoTransmission(ros::NodeHandle& nh) : last_get_data_time_(ros::Time::now())
  {
    ROS_INFO("Video transmission load.");
    custom_controller_cmd_pub_ = nh.advertise<rm_msgs::CustomControllerData>("custom_controller_data", 1);
    vt_keyboard_mouse_pub_ = nh.advertise<rm_msgs::VTKeyboardMouseData>("keyboard_mouse_data", 1);
    vt_receiver_control_pub_ = nh.advertise<rm_msgs::VTReceiverControlData>("receiver_control_data", 1);
    robot_custom_data_pub_ = nh.advertise<rm_msgs::RobotCustomData>("robot_custom_data", 1);
    robot_custom_data_2_pub_ = nh.advertise<rm_msgs::RobotCustomData2>("robot_custom_data_2", 1);
    custom_client_cmd_data_pub_ = nh.advertise<rm_msgs::CustomClientCmdData>("custom_client_cmd_data", 1);
    base_.initSerial();
  }
  void read();
  void clearRxBuffer()
  {
    rx_buffer_.clear();
    rx_len_ = 0;
  }

  ros::Publisher custom_controller_cmd_pub_, vt_keyboard_mouse_pub_, vt_receiver_control_pub_;
  ros::Publisher robot_custom_data_pub_, robot_custom_data_2_pub_, custom_client_cmd_data_pub_;

  Base base_;
  std::vector<uint8_t> rx_buffer_;
  int rx_len_ = 0;

private:
  int unpack(uint8_t* rx_data, int rx_data_len);
  int control_data_unpack(uint8_t* rx_data, int rx_data_len);
  ros::Time last_get_data_time_;
  static constexpr int k_header_length_ = 5;
  static constexpr int k_cmd_id_length_ = 2;
  static constexpr int k_tail_length_ = 2;
  static constexpr int k_min_frame_length_ = k_header_length_ + k_cmd_id_length_ + k_tail_length_;
  static constexpr int k_unpack_buffer_length_ = 512;
  static constexpr int k_max_data_length_ = k_unpack_buffer_length_ - k_min_frame_length_;
  std::array<uint8_t, k_unpack_buffer_length_> unpack_buffer_{};
};
}  // namespace rm_vt
