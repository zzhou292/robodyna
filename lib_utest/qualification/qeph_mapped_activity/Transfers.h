// SPDX-License-Identifier: MIT
#pragma once
#include <cuda_runtime.h>
#include <cstddef>

namespace qeph_activity_test {
struct Transfers {
  bool enabled = false;
  std::size_t parents = 0, calls = 0, bytes = 0, force_calls = 0, compact_calls = 0;
  const void* force_source = nullptr;
  const void* failure_source = nullptr;
  void* compact_destination = nullptr;
  cudaStream_t stream = nullptr;
};
extern Transfers transfers;
inline void Watch(std::size_t parents) {
  transfers = {};
  transfers.enabled = true;
  transfers.parents = parents;
}
} // namespace qeph_activity_test
