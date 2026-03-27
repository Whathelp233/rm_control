//
// Created by ch on 24-11-23.
//
#include "rm_vt/video_transmission.h"

namespace
{
inline uint8_t toLowerAscii(uint8_t value)
{
  if (value >= static_cast<uint8_t>('A') && value <= static_cast<uint8_t>('Z'))
    return static_cast<uint8_t>(value - static_cast<uint8_t>('A') + static_cast<uint8_t>('a'));
  return value;
}

inline bool keyMatches(uint8_t key_1, uint8_t key_2, char expected)
{
  const uint8_t expected_u8 = static_cast<uint8_t>(expected);
  return toLowerAscii(key_1) == expected_u8 || toLowerAscii(key_2) == expected_u8;
}

void mapLegacyKeyboardMouse(const rm_vt::KeyboardMouseData& keyboard_mouse_ref,
                            rm_msgs::VTKeyboardMouseData& keyboard_mouse_data)
{
  static rm_msgs::VTKeyboardMouseData last_keyboard_state;
  static bool initialized = false;
  if (!initialized)
  {
    last_keyboard_state = rm_msgs::VTKeyboardMouseData();
    initialized = true;
  }

  const uint8_t key_1 = static_cast<uint8_t>(keyboard_mouse_ref.key_value & 0x00FFu);
  const uint8_t key_2 = static_cast<uint8_t>((keyboard_mouse_ref.key_value >> 8) & 0x00FFu);
  const bool has_new_key_value = key_1 != 0u || key_2 != 0u;

  keyboard_mouse_data.mouse_x = static_cast<int16_t>(keyboard_mouse_ref.x_position);
  keyboard_mouse_data.mouse_y = static_cast<int16_t>(keyboard_mouse_ref.y_position);
  keyboard_mouse_data.mouse_z = 0;
  keyboard_mouse_data.left_button_down = (keyboard_mouse_ref.mouse_left == 1u);
  keyboard_mouse_data.right_button_down = (keyboard_mouse_ref.mouse_right == 1u);

  if (has_new_key_value)
  {
    last_keyboard_state.key_w = keyMatches(key_1, key_2, 'w');
    last_keyboard_state.key_s = keyMatches(key_1, key_2, 's');
    last_keyboard_state.key_a = keyMatches(key_1, key_2, 'a');
    last_keyboard_state.key_d = keyMatches(key_1, key_2, 'd');
    last_keyboard_state.key_q = keyMatches(key_1, key_2, 'q');
    last_keyboard_state.key_e = keyMatches(key_1, key_2, 'e');
    last_keyboard_state.key_r = keyMatches(key_1, key_2, 'r');
    last_keyboard_state.key_f = keyMatches(key_1, key_2, 'f');
    last_keyboard_state.key_g = keyMatches(key_1, key_2, 'g');
    last_keyboard_state.key_z = keyMatches(key_1, key_2, 'z');
    last_keyboard_state.key_x = keyMatches(key_1, key_2, 'x');
    last_keyboard_state.key_c = keyMatches(key_1, key_2, 'c');
    last_keyboard_state.key_v = keyMatches(key_1, key_2, 'v');
    last_keyboard_state.key_b = keyMatches(key_1, key_2, 'b');
  }

  keyboard_mouse_data.key_w = last_keyboard_state.key_w;
  keyboard_mouse_data.key_s = last_keyboard_state.key_s;
  keyboard_mouse_data.key_a = last_keyboard_state.key_a;
  keyboard_mouse_data.key_d = last_keyboard_state.key_d;
  keyboard_mouse_data.key_q = last_keyboard_state.key_q;
  keyboard_mouse_data.key_e = last_keyboard_state.key_e;
  keyboard_mouse_data.key_r = last_keyboard_state.key_r;
  keyboard_mouse_data.key_f = last_keyboard_state.key_f;
  keyboard_mouse_data.key_g = last_keyboard_state.key_g;
  keyboard_mouse_data.key_z = last_keyboard_state.key_z;
  keyboard_mouse_data.key_x = last_keyboard_state.key_x;
  keyboard_mouse_data.key_c = last_keyboard_state.key_c;
  keyboard_mouse_data.key_v = last_keyboard_state.key_v;
  keyboard_mouse_data.key_b = last_keyboard_state.key_b;

  // 0x0306 only provides two generic key slots, no dedicated Shift/Ctrl signal.
  keyboard_mouse_data.key_shift = false;
  keyboard_mouse_data.key_ctrl = false;
}
}  // namespace

namespace rm_vt
{
void VideoTransmission::read()
{
  if (!base_.serial_.available())
    return;

  rx_len_ = static_cast<int>(base_.serial_.available());
  if (rx_len_ <= 0)
    return;

  rx_buffer_.resize(static_cast<size_t>(rx_len_));
  base_.serial_.read(rx_buffer_, static_cast<size_t>(rx_len_));

  if (ros::Time::now() - last_get_data_time_ > ros::Duration(0.1))
    base_.video_transmission_is_online_ = false;

  std::array<uint8_t, k_unpack_buffer_length_> temp_buffer{};
  if (rx_len_ < k_unpack_buffer_length_)
  {
    for (int k_i = 0; k_i < k_unpack_buffer_length_ - rx_len_; ++k_i)
      temp_buffer[k_i] = unpack_buffer_[k_i + rx_len_];
    for (int k_i = 0; k_i < rx_len_; ++k_i)
      temp_buffer[k_i + k_unpack_buffer_length_ - rx_len_] = rx_buffer_[static_cast<size_t>(k_i)];
    unpack_buffer_ = temp_buffer;
  }
  else
  {
    const int offset = rx_len_ - k_unpack_buffer_length_;
    for (int k_i = 0; k_i < k_unpack_buffer_length_; ++k_i)
      unpack_buffer_[k_i] = rx_buffer_[static_cast<size_t>(offset + k_i)];
  }

  for (int k_i = 0; k_i <= k_unpack_buffer_length_ - k_min_frame_length_; ++k_i)
  {
    const int remaining_len = k_unpack_buffer_length_ - k_i;
    int frame_len = -1;
    if (unpack_buffer_[k_i] == 0xA5)
      frame_len = unpack(unpack_buffer_.data() + k_i, remaining_len);
    else if (remaining_len >= 2 && unpack_buffer_[k_i] == 0xA9 && unpack_buffer_[k_i + 1] == 0x53)
      frame_len = control_data_unpack(unpack_buffer_.data() + k_i, remaining_len);

    if (frame_len > 0)
      k_i += frame_len - 1;
  }

  clearRxBuffer();
}

int VideoTransmission::unpack(uint8_t* rx_data, int rx_data_len)
{
  if (rx_data_len < k_header_length_)
    return -1;

  uint16_t cmd_id;
  int frame_len;
  rm_vt::FrameHeader frame_header;

  memcpy(&frame_header, rx_data, k_header_length_);
  if (static_cast<bool>(base_.verifyCRC8CheckSum(rx_data, k_header_length_)))
  {
    if (frame_header.data_length > k_max_data_length_)
    {
      ROS_INFO("discard possible wrong frames, data length: %d", frame_header.data_length);
      return 0;
    }
    frame_len = frame_header.data_length + k_header_length_ + k_cmd_id_length_ + k_tail_length_;
    if (frame_len > rx_data_len || frame_len > k_unpack_buffer_length_)
      return -1;

    if (base_.verifyCRC16CheckSum(rx_data, static_cast<uint32_t>(frame_len)) == 1)
    {
      const ros::Time stamp = ros::Time::now();
      cmd_id = (rx_data[6] << 8 | rx_data[5]);
      switch (cmd_id)
      {
        case rm_vt::CUSTOM_CONTROLLER_CMD:
        {
          rm_vt::CustomControllerData custom_controller_ref;
          rm_msgs::CustomControllerData custom_controller_data;
          memcpy(&custom_controller_ref, rx_data + 7, sizeof(rm_vt::CustomControllerData));
          custom_controller_data.encoder_data[0] = 3.14 *
                                                   ((uint16_t)(custom_controller_ref.encoder1_data[0] << 8) |
                                                    (uint16_t)custom_controller_ref.encoder1_data[1]) /
                                                   18000.0;
          custom_controller_data.encoder_data[1] = 3.14 *
                                                   ((uint16_t)(custom_controller_ref.encoder2_data[0] << 8) |
                                                    (uint16_t)custom_controller_ref.encoder2_data[1]) /
                                                   18000.0;
          custom_controller_data.encoder_data[5] = 3.14 *
                                                   ((uint16_t)(custom_controller_ref.encoder3_data[0] << 8) |
                                                    (uint16_t)custom_controller_ref.encoder3_data[1]) /
                                                   18000.0;
          custom_controller_data.encoder_data[3] = 3.14 *
                                                   ((uint16_t)(custom_controller_ref.encoder4_data[0] << 8) |
                                                    (uint16_t)custom_controller_ref.encoder4_data[1]) /
                                                   18000.0;
          custom_controller_data.encoder_data[4] = 3.14 *
                                                   ((uint16_t)(custom_controller_ref.encoder5_data[0] << 8) |
                                                    (uint16_t)custom_controller_ref.encoder5_data[1]) /
                                                   18000.0;
          custom_controller_data.encoder_data[2] = 3.14 *
                                                   ((uint16_t)(custom_controller_ref.encoder6_data[0] << 8) |
                                                    (uint16_t)custom_controller_ref.encoder6_data[1]) /
                                                   18000.0;
          custom_controller_data.joystick_l_y_data = ((uint16_t)(custom_controller_ref.joystick_l_x_data[0] << 8) |
                                                      (uint16_t)custom_controller_ref.joystick_l_x_data[1]);
          custom_controller_data.joystick_l_x_data = ((uint16_t)(custom_controller_ref.joystick_l_y_data[0] << 8) |
                                                      (uint16_t)custom_controller_ref.joystick_l_y_data[1]);
          custom_controller_data.joystick_r_y_data = ((uint16_t)(custom_controller_ref.joystick_r_x_data[0] << 8) |
                                                      (uint16_t)custom_controller_ref.joystick_r_x_data[1]);
          custom_controller_data.joystick_r_x_data = ((uint16_t)(custom_controller_ref.joystick_r_y_data[0] << 8) |
                                                      (uint16_t)custom_controller_ref.joystick_r_y_data[1]);
          custom_controller_data.button_data[0] = custom_controller_ref.button1_data;
          custom_controller_data.button_data[1] = custom_controller_ref.button2_data;
          custom_controller_data.button_data[2] = custom_controller_ref.button3_data;
          custom_controller_data.button_data[3] = custom_controller_ref.button4_data;
          custom_controller_cmd_pub_.publish(custom_controller_data);
          break;
        }
        case rm_vt::KEYBOARD_MOUSE_CMD:
        {
          rm_vt::KeyboardMouseData keyboard_mouse_ref;
          rm_msgs::VTKeyboardMouseData keyboard_mouse_data;
          memcpy(&keyboard_mouse_ref, rx_data + 7, sizeof(rm_vt::KeyboardMouseData));

          mapLegacyKeyboardMouse(keyboard_mouse_ref, keyboard_mouse_data);
          vt_keyboard_mouse_pub_.publish(keyboard_mouse_data);
          break;
        }
        case rm_vt::ROBOT_TO_CUSTOM_CMD:
        {
          rm_vt::RobotToCustomData robot_to_custom_ref;
          rm_msgs::RobotCustomData robot_custom_data;
          memcpy(&robot_to_custom_ref, rx_data + 7, sizeof(rm_vt::RobotToCustomData));
          for (size_t i = 0; i < robot_custom_data.data.size(); ++i)
            robot_custom_data.data[i] = robot_to_custom_ref.data[i];
          robot_custom_data.stamp = stamp;
          robot_custom_data_pub_.publish(robot_custom_data);
          break;
        }
        case rm_vt::ROBOT_TO_CUSTOM_CMD_2:
        {
          rm_vt::RobotToCustomData2 robot_to_custom_data_2_ref;
          rm_msgs::RobotCustomData2 robot_custom_data_2;
          memcpy(&robot_to_custom_data_2_ref, rx_data + 7, sizeof(rm_vt::RobotToCustomData2));
          for (size_t i = 0; i < robot_custom_data_2.data.size(); ++i)
            robot_custom_data_2.data[i] = robot_to_custom_data_2_ref.data[i];
          robot_custom_data_2.stamp = stamp;
          robot_custom_data_2_pub_.publish(robot_custom_data_2);
          break;
        }
        case rm_vt::CUSTOM_TO_ROBOT_CMD:
        {
          rm_vt::CustomToRobotData custom_to_robot_ref;
          rm_msgs::CustomClientCmdData custom_client_cmd_data;
          memcpy(&custom_to_robot_ref, rx_data + 7, sizeof(rm_vt::CustomToRobotData));
          for (size_t i = 0; i < custom_client_cmd_data.data.size(); ++i)
            custom_client_cmd_data.data[i] = custom_to_robot_ref.data[i];
          custom_client_cmd_data.stamp = stamp;
          custom_client_cmd_data_pub_.publish(custom_client_cmd_data);
          break;
        }
        default:
          ROS_WARN("Video transmission command ID %d not found.", cmd_id);
          break;
      }
      base_.video_transmission_is_online_ = true;
      last_get_data_time_ = stamp;
      return frame_len;
    }
  }
  return -1;
}

int VideoTransmission::control_data_unpack(uint8_t* rx_data, int rx_data_len)
{
  const int frame_len = 21;
  if (rx_data_len < frame_len)
    return -1;

  if (base_.verifyCRC16CheckSum(rx_data, static_cast<uint32_t>(frame_len)) == 1)
  {
    rm_vt::ControlData control_ref;
    rm_msgs::VTReceiverControlData control_data;
    memcpy(&control_ref, rx_data + 2, sizeof(rm_vt::ControlData));
    control_data.joystick_r_x = (control_ref.joystick_r_x - 1024.0) / 660.0;
    control_data.joystick_r_y = (control_ref.joystick_r_y - 1024.0) / 660.0;
    control_data.joystick_l_y = (control_ref.joystick_l_y - 1024.0) / 660.0;
    control_data.joystick_l_x = (control_ref.joystick_l_x - 1024.0) / 660.0;
    control_data.mode_switch = control_ref.mode_switch;
    control_data.pause_button = control_ref.pause_button;
    control_data.custom_button_l = control_ref.custom_button_l;
    control_data.custom_button_r = control_ref.custom_button_r;
    control_data.wheel = (control_ref.wheel - 1024.0) / 660.0;
    control_data.trigger = control_ref.trigger;
    control_data.mouse_x = control_ref.mouse_x;
    control_data.mouse_y = control_ref.mouse_y;
    control_data.mouse_wheel = control_ref.mouse_wheel;
    control_data.mouse_left_down = control_ref.mouse_left_down;
    control_data.mouse_right_down = control_ref.mouse_right_down;
    control_data.mouse_mid_down = control_ref.mouse_mid_down;
    control_data.key_w = control_ref.key_w;
    control_data.key_s = control_ref.key_s;
    control_data.key_a = control_ref.key_a;
    control_data.key_d = control_ref.key_d;
    control_data.key_shift = control_ref.key_shift;
    control_data.key_ctrl = control_ref.key_ctrl;
    control_data.key_q = control_ref.key_q;
    control_data.key_e = control_ref.key_e;
    control_data.key_r = control_ref.key_r;
    control_data.key_f = control_ref.key_f;
    control_data.key_g = control_ref.key_g;
    control_data.key_z = control_ref.key_z;
    control_data.key_x = control_ref.key_x;
    control_data.key_c = control_ref.key_c;
    control_data.key_v = control_ref.key_v;
    control_data.key_b = control_ref.key_b;
    vt_receiver_control_pub_.publish(control_data);

    base_.video_transmission_is_online_ = true;
    last_get_data_time_ = ros::Time::now();
    return frame_len;
  }
  return -1;
}
}  // namespace rm_vt
