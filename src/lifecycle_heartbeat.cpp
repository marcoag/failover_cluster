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

#include "fail_demo/lifecycle_heartbeat.hpp"

using namespace std::chrono_literals;


constexpr std::chrono::milliseconds LEASE_DELTA = 20ms; ///< Buffer added to heartbeat to define lease.
constexpr char DEFAULT_TOPIC_NAME[] = "heartbeat";

namespace {

void print_usage()
{
    std::cout <<
        "Usage: lifecycle_heartbeat [-h] --ros-args -p heartbeat_period:=value [...]\n\n"
        "required arguments:\n"
        "\tperiod: Period in positive integer milliseconds of the heartbeat signal.\n"
        "optional arguments:\n"
        "\t-h : Print this help message." <<
        std::endl;
}

} // anonymous ns

namespace lifecycle_heartbeat
{

/// SimpleWatchdog inheriting from rclcpp_lifecycle::LifecycleNode
/**
 * Internally relies on the QoS liveliness policy provided by rmw implementation (e.g., DDS).
 * The lease passed to this watchdog has to be > the period of the heartbeat signal to account
 * for network transmission times.
 */
LifecycleHeartbeat::LifecycleHeartbeat(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("lifecycle_heartbeat", options),
        active_node_(false), topic_name_(DEFAULT_TOPIC_NAME), qos_profile_(1),
        wakeup_topic_ ("status")
{
    
    declare_parameter("heartbeat_period");
    declare_parameter("active_node");

    // Lease duration must be >= heartbeat's lease duration
    try {
        heartbeat_period_ = std::chrono::milliseconds(get_parameter("heartbeat_period").as_int());
        active_node_ = get_parameter("active_node").as_bool();
    } catch (...) {
        print_usage();
        // TODO: Update the rclcpp_components template to be able to handle
        // exceptions. Raise one here, so stack unwinding happens gracefully.
        std::exit(-1);
    }

    configure();
    
    if(active_node_) {
        RCLCPP_INFO(get_logger(), "Activation selected");
        activate();
    }
}

void LifecycleHeartbeat::timer_callback()
{

  auto message = sw_watchdog_msgs::msg::Heartbeat();
  rclcpp::Time now = this->get_clock()->now();
  message.stamp = now;
  RCLCPP_INFO(this->get_logger(), "Publishing heartbeat, sent at [%f]", now.seconds());
  publisher_->publish(message);
  
}

    /// Transition callback for state configuring
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleHeartbeat::on_configure(
    const rclcpp_lifecycle::State &)
{
  // Initialize and configure node
  qos_profile_
    .liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
    .liveliness_lease_duration(heartbeat_period_ + LEASE_DELTA)
    .deadline(heartbeat_period_ + LEASE_DELTA);

    publisher_ = this->create_publisher<sw_watchdog_msgs::msg::Heartbeat>("heartbeat", qos_profile_);
    
    status_sub_ = this->create_subscription<sw_watchdog_msgs::msg::Status>(
            wakeup_topic_,
            1,
            [this](const typename sw_watchdog_msgs::msg::Status::SharedPtr msg) -> void {
                RCLCPP_INFO(get_logger(), "Watchdog raised, self activation triggered", msg->stamp.sec);
                activate();
            });

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state activating
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleHeartbeat::on_activate(
    const rclcpp_lifecycle::State &)
{
    timer_ = this->create_wall_timer(heartbeat_period_, std::bind(&LifecycleHeartbeat::timer_callback, this));
    publisher_->on_activate();
    
    system("ros2 run failover_cluster linktime_composition --ros-args -p heartbeat_period:=200 -p watchdog_period:=220 -p active_node:=false&");

    // Starting from this point, all messages are sent to the network.
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

 
/// Transition callback for state deactivating
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleHeartbeat::on_deactivate(
    const rclcpp_lifecycle::State &)
{

    // Starting from this point, all messages are no longer sent to the network.
    publisher_->on_deactivate();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state cleaningup
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleHeartbeat::on_cleanup(
    const rclcpp_lifecycle::State &)
{
    publisher_.reset();
    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state shutting down
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleHeartbeat::on_shutdown(
    const rclcpp_lifecycle::State &state)
{
    publisher_.reset();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called from state %s.", state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
 
} // namespace sw_watchdog

RCLCPP_COMPONENTS_REGISTER_NODE(lifecycle_heartbeat::LifecycleHeartbeat)
