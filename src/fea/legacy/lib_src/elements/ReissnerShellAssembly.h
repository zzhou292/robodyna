#pragma once

#include "ReissnerShellData.h"
#include "ReissnerRotation.h"
#include "../solvers/FENodalStateView.h"
#include "../solvers/NodalForceAssembly.h"

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
  switch (AccumulateNodalForces<4>(nodes,result.force,result.couple,view)) {
    case NodalForceAssemblyStatus::Success: return ShellAssemblyStatus::kSuccess;
    case NodalForceAssemblyStatus::InvalidView: return ShellAssemblyStatus::kInvalidView;
    case NodalForceAssemblyStatus::InvalidConnectivity: return ShellAssemblyStatus::kInvalidConnectivity;
    case NodalForceAssemblyStatus::NonfiniteResult: return ShellAssemblyStatus::kNonfiniteResult;
  }
  return ShellAssemblyStatus::kInvalidView;
}
}  // namespace tl::fea::reissner
#undef TL_SHELL_HD
