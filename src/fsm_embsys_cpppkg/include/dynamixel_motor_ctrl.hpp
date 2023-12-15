// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DYNAMIXEL_MOTOR_CTRL_
#define DYNAMIXEL_MOTOR_CTRL_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "embsys_interfaces/msg/motor_pos.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"


class ReadWriteNode : public rclcpp::Node
{
public:
  using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
  using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;

  ReadWriteNode();
  void publish_present_position();
  virtual ~ReadWriteNode();

private:
  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
  rclcpp::Publisher<embsys_interfaces::msg::MotorPos>::SharedPtr get_position_publisher_;
  rclcpp::TimerBase::SharedPtr control_loop_timer;

  int present_position;
};

#endif  // DYNAMIXEL_MOTOR_CTRL_
