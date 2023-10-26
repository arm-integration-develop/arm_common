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
// Created by qiayuan on 12/28/20.
//
#include "interface/can_interface/can_bus.h"

#include <string>
#include <ros/ros.h>

namespace can_interface
{
CanBus::CanBus(const std::string& bus_name, CanDataPtr data_ptr, int thread_priority)
  : bus_name_(bus_name), data_ptr_(data_ptr)
{
  // Initialize device at can_device, false for no loop back.
  while (!socket_can_.open(bus_name, boost::bind(&CanBus::frameCallback, this, _1), thread_priority) && ros::ok())
    ros::Duration(.5).sleep();

  ROS_INFO("Successfully connected to %s.", bus_name.c_str());
}

void CanBus::start() {
    for (auto& item : *data_ptr_.id2act_data_)
    {
        if (item.second.type.find("dm") != std::string::npos)
        {
            for (int count = 0; count < 5; ++count)
            {
                can_frame frame{};
                for (int i = 0; i < 7; i++) {
                    frame.data[i] = 0xFF;
                }
                frame.data[7] = 0xFC;
                frame.can_id = item.first;
                frame.can_dlc = 8;
                socket_can_.write(&frame);
            }
        }
    }
}

void CanBus::close() {
    for (auto& item : *data_ptr_.id2act_data_)
    {
        if (item.second.type.find("dm") != std::string::npos)
        {
            can_frame frame{};
            for (int i = 0; i < 7; i++) {
                frame.data[i] = 0xFF;
            }
            frame.data[7] = 0xFD;
            frame.can_id = item.first;
            frame.can_dlc = 8;
            socket_can_.write(&frame);
        }
    }
}

void CanBus::test() {
    for (auto& item : *data_ptr_.id2act_data_)
    {
        if (item.second.type.find("dm") != std::string::npos)
        {
//       test effort
//            item.second.exe_effort = 1;

//       test pos
//            item.second.cmd_pos = 0.;
//            item.second.cmd_kp = 1.;

//       test vel
            float vel = 3;
            item.second.cmd_vel = vel;
            item.second.cmd_kd = 1;
        }
    }
    write();
}

void CanBus::write()
{
  for (auto& item : *data_ptr_.id2act_data_)
  {
    if (item.second.type.find("dm") != std::string::npos)
    {
      can_frame frame{};
      const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(item.second.type)->second;
      frame.can_id = item.first;
      frame.can_dlc = 8;
      uint16_t q_des = static_cast<int>(act_coeff.pos2act * (item.second.cmd_pos - act_coeff.pos_offset));
      uint16_t qd_des = static_cast<int>(act_coeff.vel2act * (item.second.cmd_vel - act_coeff.vel_offset));
      uint16_t kp = static_cast<int>(act_coeff.kp2act * item.second.cmd_kp);
      uint16_t kd = static_cast<int>(act_coeff.kd2act * item.second.cmd_kd);
      uint16_t tau = static_cast<int>(act_coeff.effort2act * (item.second.exe_effort - act_coeff.effort_offset));
      frame.data[0] = q_des >> 8;
      frame.data[1] = q_des & 0xFF;
      frame.data[2] = qd_des >> 4;
      frame.data[3] = ((qd_des & 0xF) << 4) | (kp >> 8);
      frame.data[4] = kp & 0xFF;
      frame.data[5] = kd >> 4;
      frame.data[6] = ((kd & 0xF) << 4) | (tau >> 8);
      frame.data[7] = tau & 0xFF;
      socket_can_.write(&frame);
    }
  }
}

void CanBus::read(ros::Time time)
{
  std::lock_guard<std::mutex> guard(mutex_);

  for (const auto& frame_stamp : read_buffer_)
  {
    can_frame frame = frame_stamp.frame;
    // Check DM motor
    if (frame.can_id == static_cast<unsigned int>(0x000))
    {
      if (data_ptr_.id2act_data_->find(frame.data[0]) != data_ptr_.id2act_data_->end())
      {
        ActData& act_data = data_ptr_.id2act_data_->find(frame.data[0])->second;
        const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(act_data.type)->second;
        if (act_data.type.find("dm") != std::string::npos)
        {  // DM Motor
          act_data.q_raw = (frame.data[1] << 8) | frame.data[2];
          uint16_t qd = (frame.data[3] << 4) | (frame.data[4] >> 4);
          uint16_t cur = ((frame.data[4] & 0xF) << 8) | frame.data[5];

          // Converter raw CAN data to position velocity and effort.
          act_data.pos = act_coeff.act2pos * static_cast<double>(act_data.q_raw) + act_coeff.pos_offset + act_data.act_offset;
          act_data.vel = act_coeff.act2vel * static_cast<double>(qd) + act_coeff.vel_offset;
          act_data.effort = act_coeff.act2effort * static_cast<double>(cur) + act_coeff.effort_offset;

//          ROS_INFO_STREAM("effort = " << act_data.effort);
//          ROS_INFO_STREAM("pos = " << act_data.pos);
//          ROS_INFO_STREAM("vel = " << act_data.vel);
          try
          {  // Duration will be out of dual 32-bit range while motor failure
            act_data.frequency = 1. / (frame_stamp.stamp - act_data.stamp).toSec();
          }
          catch (std::runtime_error& ex)
          {
          }
          act_data.stamp = frame_stamp.stamp;
          act_data.seq++;
          // Low pass filter
//          act_data.lp_filter->input(act_data.vel);
//          act_data.vel = act_data.lp_filter->output();
          continue;
        }
      }
    }
    if (frame.can_id != 0x0)
      ROS_ERROR_STREAM_ONCE("Can not find defined device, id: 0x" << std::hex << frame.can_id
                                                                  << " on bus: " << bus_name_);
  }
  read_buffer_.clear();
}

void CanBus::write(can_frame* frame)
{
  socket_can_.write(frame);
}

void CanBus::frameCallback(const can_frame& frame)
{
  std::lock_guard<std::mutex> guard(mutex_);
  CanFrameStamp can_frame_stamp{ .frame = frame, .stamp = ros::Time::now() };
  read_buffer_.push_back(can_frame_stamp);
}

}  // namespace arm_hw
