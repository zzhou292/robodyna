// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>
namespace native_gpu_copy_fault {
void Arm(std::size_t successful_host_to_device_copies) noexcept;
void Reset() noexcept;
struct Scope {
  explicit Scope(std::size_t successful_copies) noexcept { Arm(successful_copies); }
  ~Scope() { Reset(); }
  Scope(const Scope&) = delete;
  Scope& operator=(const Scope&) = delete;
};
}  // namespace native_gpu_copy_fault
