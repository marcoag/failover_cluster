// Copyright (c) 2020 Mapless AI, Inc.
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

#include "fail_demo/lifecycle_talker.hpp"

using namespace std::chrono_literals;

namespace lifecycle_talker
{

void print_usage()
{
    std::cout <<
        "Usage: lifecycle [-h] --ros-args -p active_node:=value [...]\n\n"
        "required arguments:\n"
        "\tperiod: Period in positive integer milliseconds of the heartbeat signal.\n"
        "optional arguments:\n"
        "\t-h : Print this help message." <<
        std::endl;
}
    
/// SimpleWatchdog inheriting from rclcpp_lifecycle::LifecycleNode
/**
 * Internally relies on the QoS liveliness policy provided by rmw implementation (e.g., DDS).
 * The lease passed to this watchdog has to be > the period of the heartbeat signal to account
 * for network transmission times.
 */
LifecycleTalker::LifecycleTalker(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("lifecycle_talker", options),
        active_node_(false), count_(0), wakeup_topic_ ("status_hd")
{
    declare_parameter("active_node");
    
        // Lease duration must be >= heartbeat's lease duration
    try {
        active_node_ = get_parameter("active_node").as_bool();
    } catch (...) {
        print_usage();
        // TODO: Update the rclcpp_components template to be able to handle
        // exceptions. Raise one here, so stack unwinding happens gracefully.
        std::exit(-1);
    }
    
    configure();

    if(active_node_) 
    {
        RCLCPP_INFO(get_logger(), "Activation selected");
        activate();
    }
    else
        RCLCPP_INFO(get_logger(), "No activation selected");
}

void LifecycleTalker::timer_callback()
{
  auto message = std::make_unique<std_msgs::msg::String>();
  message->data = "Hello World: " + std::to_string(++count_);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message->data.c_str());
  std::flush(std::cout);

  // Put the message into a queue to be processed by the middleware.
  // This call is non-blocking.
  publisher_->publish(std::move(message));
}

/// Transition callback for state configuring
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleTalker::on_configure(
    const rclcpp_lifecycle::State &)
{
    
    status_sub_ = this->create_subscription<sw_watchdog_msgs::msg::Status>(
            wakeup_topic_,
            10,
            [this](const typename sw_watchdog_msgs::msg::Status::SharedPtr msg) -> void {
                if(!active_node_)
                {
                    RCLCPP_INFO(get_logger(), "Watchdog raised, self activation triggered", msg->stamp.sec);
                    activate();
                }
            });
    

    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state activating
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleTalker::on_activate(
    const rclcpp_lifecycle::State &)
{
    timer_ = this->create_wall_timer(1s, std::bind(&LifecycleTalker::timer_callback, this));
    publisher_->on_activate();

    // Starting from this point, all messages are sent to the network.
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

 
/// Transition callback for state deactivating
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleTalker::on_deactivate(
    const rclcpp_lifecycle::State &)
{

    // Starting from this point, all messages are no longer sent to the network.
    publisher_->on_deactivate();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state cleaningup
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleTalker::on_cleanup(
    const rclcpp_lifecycle::State &)
{
    publisher_.reset();
    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state shutting down
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleTalker::on_shutdown(
    const rclcpp_lifecycle::State &state)
{
    publisher_.reset();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called from state %s.", state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
 
} // namespace sw_watchdog

RCLCPP_COMPONENTS_REGISTER_NODE(lifecycle_talker::LifecycleTalker)
