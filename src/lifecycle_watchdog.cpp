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

#include "fail_demo/lifecycle_watchdog.hpp"

using namespace std::chrono_literals;

constexpr char OPTION_AUTO_START[] = "--activate";
constexpr char OPTION_PUB_STATUS[] = "--publish";
constexpr char DEFAULT_TOPIC_NAME[] = "heartbeat";

namespace {

void print_usage()
{
    std::cout <<
        "Usage: lifecycle_watchdog lease [" << OPTION_AUTO_START << "] [-h]\n\n"
        "required arguments:\n"
        "\tlease: Lease in positive integer milliseconds granted to the watched entity.\n"
        "optional arguments:\n"
        "\t" << OPTION_AUTO_START << ": Start the watchdog on creation.  Defaults to false.\n"
        "\t" << OPTION_PUB_STATUS << ": Publish lease expiration of the watched entity.  "
        "Defaults to false.\n"
        "\t-h : Print this help message." <<
        std::endl;
}

} // anonymous ns

namespace lifecycle_watchdog
{

/// LifecycleWatchdog inheriting from rclcpp_lifecycle::LifecycleNode
/**
 * Internally relies on the QoS liveliness policy provided by rmw implementation (e.g., DDS).
 * The lease passed to this watchdog has to be > the period of the heartbeat signal to account
 * for network transmission times.
 */
LifecycleWatchdog::LifecycleWatchdog(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("lifecycle_watchdog", options),
        active_node_(false), enable_pub_(true), topic_name_(DEFAULT_TOPIC_NAME), qos_profile_(10)
{
    
    declare_parameter("watchdog_period");
    declare_parameter("active_node");

    // Lease duration must be >= heartbeat's lease duration
    try {
        lease_duration_ = std::chrono::milliseconds(get_parameter("watchdog_period").as_int());
        active_node_ = get_parameter("active_node").as_bool();
    } catch (...) {
        print_usage();
        // TODO: Update the rclcpp_components template to be able to handle
        // exceptions. Raise one here, so stack unwinding happens gracefully.
        std::exit(-1);
    }

    if(!active_node_) {
        configure();
        activate();
    }
}

/// Publish lease expiry of the watched entity
void LifecycleWatchdog::publish_status()
{
    auto msg = std::make_unique<sw_watchdog_msgs::msg::Status>();
    rclcpp::Time now = this->get_clock()->now();
    msg->stamp = now;
    msg->missed_number = 1;

    // Print the current state for demo purposes
    if (!status_pub_->is_activated()) {
        RCLCPP_INFO(get_logger(),
                    "Lifecycle publisher is currently inactive. Messages are not published.");
    } else {
        RCLCPP_INFO(get_logger(),
                    "Publishing lease expiry (missed count: %u) at [%f]",
                    msg->missed_number, now.seconds());
    }

    // Only if the publisher is in an active state, the message transfer is
    // enabled and the message actually published.
    status_pub_->publish(std::move(msg));
}

    /// Transition callback for state configuring
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleWatchdog::on_configure(
    const rclcpp_lifecycle::State &)
{
    // Initialize and configure node
    qos_profile_
        .liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
        .liveliness_lease_duration(lease_duration_);

    heartbeat_sub_options_.event_callbacks.liveliness_callback =
        [this](rclcpp::QOSLivelinessChangedInfo &event) -> void {
            printf("Reader Liveliness changed event: \n");
            printf("  alive_count: %d\n", event.alive_count);
            printf("  not_alive_count: %d\n", event.not_alive_count);
            printf("  alive_count_change: %d\n", event.alive_count_change);
            printf("  not_alive_count_change: %d\n", event.not_alive_count_change);
            if(event.alive_count == 0) {
                publish_status();
                // Transition lifecycle to deactivated state
                deactivate();
                // Activate talker and heartbeat
            }
        };

    if(enable_pub_)
        status_pub_ = create_publisher<sw_watchdog_msgs::msg::Status>("status", 1); /* QoS history_depth */

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state activating
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleWatchdog::on_activate(
    const rclcpp_lifecycle::State &)
{
    if(!heartbeat_sub_) {
        heartbeat_sub_ = create_subscription<sw_watchdog_msgs::msg::Heartbeat>(
            topic_name_,
            qos_profile_,
            [this](const typename sw_watchdog_msgs::msg::Heartbeat::SharedPtr msg) -> void {
                RCLCPP_INFO(get_logger(), "Watchdog raised, heartbeat sent at [%d.x]", msg->stamp.sec);
            },
            heartbeat_sub_options_);
    }

    // Starting from this point, all messages are sent to the network.
    if (enable_pub_)
        status_pub_->on_activate();

    // Starting from this point, all messages are sent to the network.
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state deactivating
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleWatchdog::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    heartbeat_sub_.reset(); // XXX there does not seem to be a 'deactivate' for subscribers.
    heartbeat_sub_ = nullptr;

    // Starting from this point, all messages are no longer sent to the network.
    if(enable_pub_)
        status_pub_->on_deactivate();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state cleaningup
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleWatchdog::on_cleanup(
    const rclcpp_lifecycle::State &)
{
    status_pub_.reset();
    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state shutting down
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleWatchdog::on_shutdown(
    const rclcpp_lifecycle::State &state)
{
    heartbeat_sub_.reset();
    heartbeat_sub_ = nullptr;
    status_pub_.reset();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called from state %s.", state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // namespace sw_watchdog

RCLCPP_COMPONENTS_REGISTER_NODE(lifecycle_watchdog::LifecycleWatchdog)
