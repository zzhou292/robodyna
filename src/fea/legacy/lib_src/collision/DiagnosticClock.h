// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstdint>
#include <ctime>

namespace tlfea::contact::diagnostic {
// Private numerical owners share only this clock primitive, never mechanics.
// A substituted reader is used by qualification; production uses Nanoseconds.
struct Clock {
  using Read = bool (*)(void*, std::uint64_t*) noexcept;
  Read read = nullptr;
  void* context = nullptr;
};
inline bool Nanoseconds(void*, std::uint64_t* output) noexcept {
  timespec value{};
  if (clock_gettime(CLOCK_MONOTONIC, &value) != 0 || value.tv_sec < 0 ||
      value.tv_nsec < 0 || value.tv_nsec >= 1000000000) return false;
  const auto seconds = static_cast<std::uint64_t>(value.tv_sec);
  const auto fraction = static_cast<std::uint64_t>(value.tv_nsec);
  if (seconds > (UINT64_MAX - fraction) / 1000000000) return false;
  *output = seconds * 1000000000 + fraction;
  return true;
}
}  // namespace tlfea::contact::diagnostic
