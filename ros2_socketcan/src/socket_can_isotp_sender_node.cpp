// Copyright 2021 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include "ros2_socketcan/socket_can_isotp_sender_node.hpp"
#include "ros2_socketcan/socket_can_common.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;

namespace drivers
{
namespace socketcan
{
SocketCanIsotpSenderNode::SocketCanIsotpSenderNode(rclcpp::NodeOptions options)
: lc::LifecycleNode("socket_can_isotp_sender_node", options)
{
  interface_ = this->declare_parameter("interface", "can0");
  tx_id_ = static_cast<uint32_t>(this->declare_parameter("tx_id", 0x600));
  rx_id_ = static_cast<uint32_t>(this->declare_parameter("rx_id", 0x601));
  double timeout_sec = this->declare_parameter("timeout_sec", 0.01);
  timeout_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timeout_sec));

  RCLCPP_INFO(this->get_logger(), "interface: %s", interface_.c_str());
  RCLCPP_INFO(this->get_logger(), "tx_id: 0x%X", tx_id_);
  RCLCPP_INFO(this->get_logger(), "rx_id: 0x%X", rx_id_);
  RCLCPP_INFO(this->get_logger(), "timeout(s): %f", timeout_sec);
}

LNI::CallbackReturn SocketCanIsotpSenderNode::on_configure(const lc::State & state)
{
  (void)state;

  try {
    sender_ = std::make_unique<SocketCanIsotpSender>(interface_, tx_id_, rx_id_);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "Error opening ISO-TP sender: %s - %s",
      interface_.c_str(), ex.what());
    return LNI::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(this->get_logger(), "ISO-TP Sender successfully configured.");

  isotp_frames_sub_ = this->create_subscription<ros2_socketcan_msgs::msg::FdFrame>(
    "to_can_bus_isotp", 500, std::bind(
      &SocketCanIsotpSenderNode::on_isotp_frame, this,
      std::placeholders::_1));

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanIsotpSenderNode::on_activate(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(this->get_logger(), "ISO-TP Sender activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanIsotpSenderNode::on_deactivate(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(this->get_logger(), "ISO-TP Sender deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanIsotpSenderNode::on_cleanup(const lc::State & state)
{
  (void)state;
  isotp_frames_sub_.reset();
  RCLCPP_DEBUG(this->get_logger(), "ISO-TP Sender cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanIsotpSenderNode::on_shutdown(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(this->get_logger(), "ISO-TP Sender shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

void SocketCanIsotpSenderNode::on_isotp_frame(
  const ros2_socketcan_msgs::msg::FdFrame::SharedPtr msg)
{
  if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
    try {
      sender_->send(msg->data.data<void>(), msg->data.size(), timeout_ns_);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Error sending ISO-TP message: %s - %s",
        interface_.c_str(), ex.what());
      return;
    }
  }
}

}  // namespace socketcan
}  // namespace drivers

RCLCPP_COMPONENTS_REGISTER_NODE(drivers::socketcan::SocketCanIsotpSenderNode)
