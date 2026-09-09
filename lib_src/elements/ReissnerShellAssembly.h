#pragma once

#include "ReissnerShellData.h"
#include "ReissnerRotation.h"
#include "../solvers/FENodalStateView.h"

#if defined(__CUDACC__)
#define TL_SHELL_HD __host__ __device__
#else
#define TL_SHELL_HD
#endif

namespace tl::fea::reissner {
enum class ShellAssemblyStatus { kSuccess, kInvalidView, kInvalidConnectivity, kNonfiniteResult };

// Add ONE completed Q4 result to the existing shared physical-node force view.
// This is a serialized contributor, not an atomic parallel scatter or a state
// owner. Inputs/pointees reside in one memory space; all six writable arrays
// contain node_count doubles and must be mutually disjoint and not overlap
// input objects. The caller validates epoch/owner and clears assembly ONCE.
// Multiple contributions to shared nodes add; no mass or time is updated.
// Failure preserves this contribution's destination values. A coordinator
// still discards whole trial assembly if any other contributor fails.
TL_SHELL_HD inline ShellAssemblyStatus AccumulateShellForces(
    const std::size_t nodes[4], const ShellResult& result, DeviceNodalForceView view) {
  if (!nodes || !view.node_count || !view.force_x || !view.force_y || !view.force_z ||
      !view.couple_x || !view.couple_y || !view.couple_z) return ShellAssemblyStatus::kInvalidView;
  double* arrays[6] = {view.force_x, view.force_y, view.force_z, view.couple_x, view.couple_y, view.couple_z};
  for (unsigned component = 0; component < 6; ++component)
    for (unsigned other = 0; other < component; ++other)
      if (arrays[component] == arrays[other]) return ShellAssemblyStatus::kInvalidView;
  double candidate[4][6];
  for (unsigned n = 0; n < 4; ++n) {
    if (nodes[n] >= view.node_count) return ShellAssemblyStatus::kInvalidConnectivity;
    for (unsigned other = 0; other < n; ++other)
      if (nodes[n] == nodes[other]) return ShellAssemblyStatus::kInvalidConnectivity;
    for (unsigned c = 0; c < 6; ++c) {
      const auto vector = c < 3 ? result.force[n] : result.couple[n];
      candidate[n][c] = arrays[c][nodes[n]] + detail::Component(vector, c % 3);
      if (!detail::Finite(candidate[n][c])) return ShellAssemblyStatus::kNonfiniteResult;
    }
  }
  for (unsigned n = 0; n < 4; ++n)
    for (unsigned c = 0; c < 6; ++c) arrays[c][nodes[n]] = candidate[n][c];
  return ShellAssemblyStatus::kSuccess;
}
}  // namespace tl::fea::reissner
#undef TL_SHELL_HD
