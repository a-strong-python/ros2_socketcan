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
/// \file
/// \brief This file defines a class for ISO-TP socket receiving
#ifndef ROS2_SOCKETCAN__SOCKET_CAN_ISOTP_RECEIVER_HPP_
#define ROS2_SOCKETCAN__SOCKET_CAN_ISOTP_RECEIVER_HPP_

#include <chrono>
#include <cstdint>
#include <string>

#include "ros2_socketcan/visibility_control.hpp"
#include "ros2_socketcan/socket_can_id.hpp"

namespace drivers
{
namespace socketcan
{

/// Simple RAII wrapper around an ISO-TP CAN receiver
class SOCKETCAN_PUBLIC SocketCanIsotpReceiver
{
public:
  /// Constructor
  /// \param[in] interface The CAN interface name (e.g. "can0")
  /// \param[in] tx_id The CAN ID used for transmitting ISO-TP flow control frames
  /// \param[in] rx_id The CAN ID used for receiving ISO-TP frames
  explicit SocketCanIsotpReceiver(
    const std::string & interface,
    uint32_t tx_id,
    uint32_t rx_id);
  /// Destructor
  ~SocketCanIsotpReceiver() noexcept;

  /// Receive ISO-TP data
  /// \param[out] data A buffer to be written with data bytes.
  ///                  Must be at least MAX_ISOTP_DATA_LENGTH bytes in size
  /// \param[in] timeout Maximum duration to wait for data on the file descriptor
  /// \return The number of bytes received
  /// \throw SocketCanTimeout On timeout
  /// \throw std::runtime_error on other errors
  std::size_t receive(
    void * const data,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero()) const;

private:
  // Wait for file descriptor to be available to read data via select()
  SOCKETCAN_LOCAL void wait(const std::chrono::nanoseconds timeout) const;

  int32_t m_file_descriptor;
};  // class SocketCanIsotpReceiver

}  // namespace socketcan
}  // namespace drivers

#endif  // ROS2_SOCKETCAN__SOCKET_CAN_ISOTP_RECEIVER_HPP_
