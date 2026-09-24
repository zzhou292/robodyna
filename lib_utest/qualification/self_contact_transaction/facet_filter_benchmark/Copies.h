// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>
namespace facet_filter_benchmark {
struct Copies {
  std::size_t host_to_device_calls = 0, device_to_host_calls = 0;
  std::size_t host_to_device_bytes = 0, device_to_host_bytes = 0;
  std::size_t minimum_query_pairs = 0, maximum_query_pairs = 0;
  bool unexpected_copy_shape = false;
};
void BeginCopyObservation() noexcept;
Copies EndCopyObservation() noexcept;
struct CopyScope {
  CopyScope() noexcept { BeginCopyObservation(); }
  ~CopyScope() { EndCopyObservation(); }
  CopyScope(const CopyScope&) = delete;
  CopyScope& operator=(const CopyScope&) = delete;
};
}  // namespace facet_filter_benchmark
