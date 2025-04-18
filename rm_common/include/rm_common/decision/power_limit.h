/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by bruce on 2021/7/28.
//

#pragma once

#include <ros/ros.h>
#include <rm_msgs/ChassisCmd.h>
#include <rm_msgs/GameStatus.h>
#include <rm_msgs/GameRobotStatus.h>
#include <rm_msgs/PowerHeatData.h>
#include <rm_msgs/PowerManagementSampleAndStatusData.h>

namespace rm_common
{
class PowerLimit
{
public:
  PowerLimit(ros::NodeHandle& nh)

  {
    if (!nh.getParam("safety_power", safety_power_))//获取安全频率
      ROS_ERROR("Safety power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("capacitor_threshold", capacitor_threshold_))//超电阈值
      ROS_ERROR("Capacitor threshold no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("charge_power", charge_power_))//充能功率
      ROS_ERROR("Charge power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("extra_power", extra_power_))//额外功率
      ROS_ERROR("Extra power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("burst_power", burst_power_))//超电功率
      ROS_ERROR("Burst power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("power_gain", power_gain_))//功率增益
      ROS_ERROR("power gain no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("buffer_threshold", buffer_threshold_))//缓冲增益？
      ROS_ERROR("buffer threshold no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("is_new_capacitor", is_new_capacitor_))//新超电（400w那个）
      ROS_ERROR("is_new_capacitor no defined (namespace: %s)", nh.getNamespace().c_str());
  }
  typedef enum
  {
    CHARGE = 0,//充能
    BURST = 1,//超电
    NORMAL = 2,//正常
    ALLOFF = 3,
    TEST = 4,//测试
  } Mode;

  void updateSafetyPower(int safety_power)
  {
    if (safety_power > 0)//确定没断电
      safety_power_ = safety_power;
    ROS_INFO("update safety power: %d", safety_power);
  }
  void updateState(uint8_t state)
  {
    if (!capacitor_is_on_)//没开超电
      expect_state_ = ALLOFF;
    else
      expect_state_ = state;
  }
  void updateCapSwitchState(bool state)
  {
    capacitor_is_on_ = state;
  }
  void setGameRobotData(const rm_msgs::GameRobotStatus data)
  {
    robot_id_ = data.robot_id;
    chassis_power_limit_ = data.chassis_power_limit;
  }
  void setChassisPowerBuffer(const rm_msgs::PowerHeatData data)
  {
    chassis_power_buffer_ = data.chassis_power_buffer;
  }
  void setCapacityData(const rm_msgs::PowerManagementSampleAndStatusData data)
  {
    capacity_is_online_ = ros::Time::now() - data.stamp < ros::Duration(0.3);
    cap_energy_ = data.capacity_remain_charge;
    cap_state_ = data.state_machine_running_state;
  }
  void setRefereeStatus(bool status)
  {
    referee_is_online_ = status;
  }

  uint8_t getState()
  {
    return expect_state_;
  }
  void setLimitPower(rm_msgs::ChassisCmd& chassis_cmd, bool is_gyro)//设置功率上限
  {
    if (robot_id_ == rm_msgs::GameRobotStatus::BLUE_ENGINEER || robot_id_ == rm_msgs::GameRobotStatus::RED_ENGINEER)//如果是工程是直接跑满的
      chassis_cmd.power_limit = 400;
    else
    {  // standard and hero
      if (referee_is_online_)
      {
        if (capacity_is_online_)
        {
          if (expect_state_ == ALLOFF)
            normal(chassis_cmd);
          else
          {
            if (chassis_power_limit_ > burst_power_)
              chassis_cmd.power_limit = burst_power_;
            else
            {
              switch (is_new_capacitor_ ? expect_state_ : cap_state_)
              {
                case NORMAL:
                  normal(chassis_cmd);
                  break;
                case BURST:
                  burst(chassis_cmd, is_gyro);
                  break;
                case CHARGE:
                  charge(chassis_cmd);
                  break;
                default:
                  zero(chassis_cmd);
                  break;
              }
            }
          }
        }
        else
          normal(chassis_cmd);
      }
      else
        chassis_cmd.power_limit = safety_power_;
    }
  }

private:
  void charge(rm_msgs::ChassisCmd& chassis_cmd)
  {
    chassis_cmd.power_limit = chassis_power_limit_ * 0.70;
  }
  void normal(rm_msgs::ChassisCmd& chassis_cmd)
  {
    double buffer_energy_error = chassis_power_buffer_ - buffer_threshold_;
    double plus_power = buffer_energy_error * power_gain_;
    chassis_cmd.power_limit = chassis_power_limit_ + plus_power;
    // TODO:Add protection when buffer<5
  }
  void zero(rm_msgs::ChassisCmd& chassis_cmd)
  {
    chassis_cmd.power_limit = 0.0;
  }
  void burst(rm_msgs::ChassisCmd& chassis_cmd, bool is_gyro)
  {
    if (cap_state_ != ALLOFF && cap_energy_ > capacitor_threshold_)
    {
      if (is_gyro)
        chassis_cmd.power_limit = chassis_power_limit_ + extra_power_;
      else
        chassis_cmd.power_limit = burst_power_;
    }
    else
      expect_state_ = NORMAL;
  }

  int chassis_power_buffer_;
  int robot_id_, chassis_power_limit_;
  float cap_energy_;
  double safety_power_{};
  double capacitor_threshold_{};
  double charge_power_{}, extra_power_{}, burst_power_{};
  double buffer_threshold_{};
  double power_gain_{};
  bool is_new_capacitor_{ false };
  uint8_t expect_state_{}, cap_state_{};
  bool capacitor_is_on_{ true };

  bool referee_is_online_{ false };
  bool capacity_is_online_{ false };
};
}  // namespace rm_common
