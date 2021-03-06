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

#ifndef FAILOVER__LIFECYCLE_HEARTBEAT_HPP_
#define FAILOVER__LIFECYCLE_HEARTBEAT_HPP_

#include <chrono>

#include "fail_demo/visibility_control.h"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "rclcpp_components/register_node_macro.hpp"

#include "sw_watchdog_msgs/msg/heartbeat.hpp"
#include "sw_watchdog_msgs/msg/status.hpp"

#include "std_msgs/msg/string.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace lifecycle_heartbeat
{

class LifecycleHeartbeat : public rclcpp_lifecycle::LifecycleNode
{
public:
  COMPOSITION_PUBLIC

  explicit LifecycleHeartbeat(const rclcpp::NodeOptions& options);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_cleanup(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_shutdown(const rclcpp_lifecycle::State &state);

private: 

    void timer_callback();
  
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sw_watchdog_msgs::msg::Heartbeat>> publisher_ = nullptr;
    rclcpp::Subscription<sw_watchdog_msgs::msg::Status>::SharedPtr status_sub_ = nullptr;
    rclcpp::Subscription<sw_watchdog_msgs::msg::Status>::SharedPtr subscription_ = nullptr;
    
    bool active_node_;
    const std::string heartbeat_topic_;
    const std::string wakeup_topic_;
    const std::string wakeup_watchdog_topic_;
    const std::string watchdog_heartbeat_topic_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::QoS qos_profile_;
    std::chrono::milliseconds heartbeat_period_;
};

}  // namespace lifecycle_heartbeat

#endif  // COMPOSITION__LIFECYCLE_HEARTBEAT_HPP_
