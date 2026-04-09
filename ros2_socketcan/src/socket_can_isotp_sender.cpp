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

#include "ros2_socketcan/socket_can_common.hpp"
#include "ros2_socketcan/socket_can_isotp_sender.hpp"

#include <unistd.h>  // for close()
#include <sys/select.h>
#include <sys/socket.h>

#include <cstring>
#include <chrono>
#include <stdexcept>
#include <string>

namespace drivers
{
namespace socketcan
{

////////////////////////////////////////////////////////////////////////////////
SocketCanIsotpSender::SocketCanIsotpSender(
  const std::string & interface,
  uint32_t tx_id,
  uint32_t rx_id)
: m_file_descriptor{bind_isotp_socket(interface, tx_id, rx_id)}
{
}

////////////////////////////////////////////////////////////////////////////////
SocketCanIsotpSender::~SocketCanIsotpSender() noexcept
{
  (void)close(m_file_descriptor);
}

////////////////////////////////////////////////////////////////////////////////
void SocketCanIsotpSender::wait(const std::chrono::nanoseconds timeout) const
{
  if (decltype(timeout)::zero() < timeout) {
    auto c_timeout = to_timeval(timeout);
    auto write_set = single_set(m_file_descriptor);
    // Wait
    if (0 == select(m_file_descriptor + 1, NULL, &write_set, NULL, &c_timeout)) {
      throw SocketCanTimeout{"ISO-TP Send Timeout"};
    }
    if (!FD_ISSET(m_file_descriptor, &write_set)) {
      throw SocketCanTimeout{"ISO-TP Send timeout"};
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void SocketCanIsotpSender::send(
  const void * const data,
  const std::size_t length,
  const std::chrono::nanoseconds timeout) const
{
  if (length > MAX_ISOTP_DATA_LENGTH) {
    throw std::domain_error{"Size is too large to send via ISO-TP"};
  }

  // Use select call on positive timeout
  wait(timeout);

  // Actually send the data (ISO-TP uses simple write, no CAN frame header needed)
  const auto bytes_sent = write(m_file_descriptor, data, length);
  if (bytes_sent < 0) {
    throw std::runtime_error{strerror(errno)};
  }
}

}  // namespace socketcan
}  // namespace drivers
