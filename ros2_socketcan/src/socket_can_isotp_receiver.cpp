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
#include "ros2_socketcan/socket_can_isotp_receiver.hpp"

#include <unistd.h>  // for close(), read()
#include <sys/select.h>
#include <sys/socket.h>

#include <cerrno>
#include <cstring>
#include <chrono>
#include <stdexcept>
#include <string>

namespace drivers
{
namespace socketcan
{

////////////////////////////////////////////////////////////////////////////////
SocketCanIsotpReceiver::SocketCanIsotpReceiver(
  const std::string & interface,
  uint32_t tx_id,
  uint32_t rx_id)
: m_file_descriptor{bind_isotp_socket(interface, tx_id, rx_id)}
{
}

////////////////////////////////////////////////////////////////////////////////
SocketCanIsotpReceiver::~SocketCanIsotpReceiver() noexcept
{
  (void)close(m_file_descriptor);
}

////////////////////////////////////////////////////////////////////////////////
void SocketCanIsotpReceiver::wait(const std::chrono::nanoseconds timeout) const
{
  if (decltype(timeout)::zero() < timeout) {
    auto c_timeout = to_timeval(timeout);
    auto read_set = single_set(m_file_descriptor);
    // Wait
    if (0 == select(m_file_descriptor + 1, &read_set, NULL, NULL, &c_timeout)) {
      throw SocketCanTimeout{"ISO-TP Receive Timeout"};
    }
    if (!FD_ISSET(m_file_descriptor, &read_set)) {
      throw SocketCanTimeout{"ISO-TP Receive timeout"};
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
std::size_t SocketCanIsotpReceiver::receive(
  void * const data,
  const std::chrono::nanoseconds timeout) const
{
  wait(timeout);

  // ISO-TP uses simple read; the kernel handles reassembly
  const auto nbytes = read(m_file_descriptor, data, MAX_ISOTP_DATA_LENGTH);
  if (nbytes < 0) {
    throw std::runtime_error{strerror(errno)};
  }

  return static_cast<std::size_t>(nbytes);
}

}  // namespace socketcan
}  // namespace drivers
