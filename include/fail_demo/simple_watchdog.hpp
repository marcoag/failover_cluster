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

#ifndef FAILOVER__WATCHDOG_COMPONENT_HPP_
#define FAILOVER__WATCHDOG_COMPONENT_HPP_

#include <chrono>
#include <atomic>
#include <iostream>

#include "fail_demo/visibility_control.h"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "rclcpp_components/register_node_macro.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "rcutils/logging_macros.h"

#include "sw_watchdog_msgs/msg/heartbeat.hpp"
#include "sw_watchdog_msgs/msg/status.hpp"

namespace simple_watchdog
{

class SimpleWatchdog : public rclcpp_lifecycle::LifecycleNode
{
public:
    COMPOSITION_PUBLIC
    explicit SimpleWatchdog(const rclcpp::NodeOptions& options);
    void publish_status();
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &state);
private:
    /// The lease duration granted to the remote (heartbeat) publisher
    std::chrono::milliseconds lease_duration_;
    rclcpp::Subscription<sw_watchdog_msgs::msg::Heartbeat>::SharedPtr heartbeat_sub_ = nullptr;
    /// Publish lease expiry for the watched entity
    // By default, a lifecycle publisher is inactive by creation and has to be activated to publish.
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sw_watchdog_msgs::msg::Status>> status_pub_ = nullptr;
    /// Whether to enable the watchdog on startup. Otherwise, lifecycle transitions have to be raised.
    bool autostart_;
    /// Whether a lease expiry should be published
    bool enable_pub_;
    /// Topic name for heartbeat signal by the watched entity
    const std::string topic_name_;
    rclcpp::QoS qos_profile_;
    rclcpp::SubscriptionOptions heartbeat_sub_options_;
};

}
#endif  // COMPOSITION__WATCHDOG_COMPONENT_HPP_
