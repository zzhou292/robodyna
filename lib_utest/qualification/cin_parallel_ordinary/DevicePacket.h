// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Packet.h"
#include <memory>
namespace tl::fea::cin_parallel_test {
struct DeviceDelete {
  void operator()(void*) const noexcept;
};
class DevicePacket {
 public:
  explicit DevicePacket(Packet&);
  DevicePacket(const DevicePacket&) = delete;
  DevicePacket& operator=(const DevicePacket&) = delete;
  void Run(bool parallel);
  void Download(Packet&) const;
 private:
  void* Upload(const void*, std::size_t);
  std::vector<std::unique_ptr<void, DeviceDelete>> allocations_;
  cin_advance::Input input_;
};
} // namespace tl::fea::cin_parallel_test
