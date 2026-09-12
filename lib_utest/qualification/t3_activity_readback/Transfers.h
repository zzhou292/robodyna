// SPDX-License-Identifier: MIT
#pragma once
#include <cuda_runtime.h>
#include <cstddef>

namespace t3_readback_test {
enum class Fault { None, ForceNonfinite, PointThickness, Both, RawPointFlag };
struct Transfers {
  bool enabled = false;
  std::size_t parents = 0, copies = 0, bytes = 0, syncs = 0, force_copies = 0, fail_copy = 0;
  void* force_host = nullptr;
  const void* force_device = nullptr;
  cudaStream_t stream = nullptr;
  Fault fault = Fault::None;
};
extern Transfers transfers;
inline void Watch(std::size_t count, Fault fault = Fault::None) {
  transfers = {};
  transfers.enabled = true;
  transfers.parents = count;
  transfers.fault = fault;
}
} // namespace t3_readback_test
