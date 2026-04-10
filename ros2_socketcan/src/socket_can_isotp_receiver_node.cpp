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

#include "ros2_socketcan/socket_can_isotp_receiver_node.hpp"
#include "ros2_socketcan/socket_can_common.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;
using namespace std::chrono_literals;

namespace drivers
{
namespace socketcan
{
SocketCanIsotpReceiverNode::SocketCanIsotpReceiverNode(rclcpp::NodeOptions options)
: lc::LifecycleNode("socket_can_isotp_receiver_node", options)
{
  interface_ = this->declare_parameter("interface", "can0");
  tx_id_ = static_cast<uint32_t>(this->declare_parameter("tx_id", 0x601));
  rx_id_ = static_cast<uint32_t>(this->declare_parameter("rx_id", 0x600));
  double interval_sec = this->declare_parameter("interval_sec", 0.01);
  interval_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(interval_sec));

  RCLCPP_INFO(this->get_logger(), "interface: %s", interface_.c_str());
  RCLCPP_INFO(this->get_logger(), "tx_id: 0x%X", tx_id_);
  RCLCPP_INFO(this->get_logger(), "rx_id: 0x%X", rx_id_);
  RCLCPP_INFO(this->get_logger(), "interval(s): %f", interval_sec);
}

LNI::CallbackReturn SocketCanIsotpReceiverNode::on_configure(const lc::State & state)
{
  (void)state;

  try {
    receiver_ = std::make_unique<SocketCanIsotpReceiver>(interface_, tx_id_, rx_id_);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "Error opening ISO-TP receiver: %s - %s",
      interface_.c_str(), ex.what());
    return LNI::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(this->get_logger(), "ISO-TP Receiver successfully configured.");

  isotp_frames_pub_ =
    this->create_publisher<ros2_socketcan_msgs::msg::FdFrame>("from_can_bus_isotp", 500);

  running_.store(true);
  receiver_thread_ = std::make_unique<std::thread>(&SocketCanIsotpReceiverNode::receive, this);

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanIsotpReceiverNode::on_activate(const lc::State & state)
{
  (void)state;
  isotp_frames_pub_->on_activate();
  RCLCPP_DEBUG(this->get_logger(), "ISO-TP Receiver activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanIsotpReceiverNode::on_deactivate(const lc::State & state)
{
  (void)state;
  isotp_frames_pub_->on_deactivate();
  RCLCPP_DEBUG(this->get_logger(), "ISO-TP Receiver deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanIsotpReceiverNode::on_cleanup(const lc::State & state)
{
  (void)state;
  running_.store(false);
  if (receiver_thread_ && receiver_thread_->joinable()) {
    receiver_thread_->join();
  }
  isotp_frames_pub_.reset();
  RCLCPP_DEBUG(this->get_logger(), "ISO-TP Receiver cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanIsotpReceiverNode::on_shutdown(const lc::State & state)
{
  (void)state;
  running_.store(false);
  if (receiver_thread_ && receiver_thread_->joinable()) {
    receiver_thread_->join();
  }
  RCLCPP_DEBUG(this->get_logger(), "ISO-TP Receiver shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

void SocketCanIsotpReceiverNode::receive()
{
  ros2_socketcan_msgs::msg::FdFrame isotp_frame_msg(
    rosidl_runtime_cpp::MessageInitialization::ZERO);
  isotp_frame_msg.header.frame_id = "can";

  while (rclcpp::ok() && running_.load()) {
    if (this->get_current_state().id() != State::PRIMARY_STATE_ACTIVE) {
      std::this_thread::sleep_for(100ms);
      continue;
    }

    // Use a local buffer for the raw ISO-TP read, since
    // FdFrame.data is bounded to 64 bytes but ISO-TP can
    // deliver up to MAX_ISOTP_DATA_LENGTH bytes.
    uint8_t raw_buf[MAX_ISOTP_DATA_LENGTH];

    std::size_t nbytes = 0;
    try {
      nbytes = receiver_->receive(raw_buf, interval_ns_);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Error receiving ISO-TP message: %s - %s",
        interface_.c_str(), ex.what());
      continue;
    }

    // Cap to FdFrame max capacity (64 bytes)
    const std::size_t copy_len =
      std::min(nbytes, MAX_FD_DATA_LENGTH);
    if (nbytes > MAX_FD_DATA_LENGTH) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ISO-TP payload (%zu bytes) exceeds FdFrame "
        "capacity (%zu bytes), truncating",
        nbytes, MAX_FD_DATA_LENGTH);
    }

    isotp_frame_msg.data.resize(copy_len);
    std::memcpy(
      isotp_frame_msg.data.data<void>(),
      raw_buf, copy_len);
    isotp_frame_msg.header.stamp = this->now();
    isotp_frame_msg.id = rx_id_;
    isotp_frame_msg.is_extended = false;
    isotp_frame_msg.is_error = false;
    isotp_frame_msg.len =
      static_cast<uint8_t>(copy_len);
    isotp_frames_pub_->publish(
      std::move(isotp_frame_msg));
  }
}

}  // namespace socketcan
}  // namespace drivers

RCLCPP_COMPONENTS_REGISTER_NODE(drivers::socketcan::SocketCanIsotpReceiverNode)
