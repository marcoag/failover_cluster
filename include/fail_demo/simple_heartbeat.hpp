// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef FAILOVER__HEARTBEAT_COMPONENT_HPP_
#define FAILOVER__HEARTBEAT_COMPONENT_HPP_

#include <chrono>

#include "fail_demo/visibility_control.h"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "rclcpp_components/register_node_macro.hpp"

#include "sw_watchdog_msgs/msg/heartbeat.hpp"

namespace simple_heartbeat
{

class SimpleHeartbeat : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC

  explicit SimpleHeartbeat(rclcpp::NodeOptions options);

private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sw_watchdog_msgs::msg::Heartbeat>::SharedPtr publisher_;

};

}  // namespace composition

#endif  // COMPOSITION__TALKER_COMPONENT_HPP_
